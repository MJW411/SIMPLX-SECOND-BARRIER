"""안전 관리자 — 판단은 여기 한 곳에서만 한다.

상태 우선순위: 화재 > 정지(사람 근접·사라짐·고장) > 감속 > 컨베이어 이상 > 정상.
- 20Hz로 판단해 실행기에 "움직여도 됨" 허가(속도 배율)를 계속 보낸다.
  이 스레드가 멎으면 허가가 끊겨 로봇이 선다.
- 정지는 걸리면 유지(latched)된다. 자동 재가동은 없다. 관리자가 재가동을 누르면
  그 순간 상태(사람 없음, 센서·인식 정상, 미해결 사라짐 없음)를 다시 검사하고 해제한다.
- 화재: 출입문 개방(아두이노가 먼저 스스로 연다), 로봇은 홈으로 복귀하되 사람이 가까우면
  멈추고 미해결 사라짐이 있으면 복귀하지 않는다. 관리자 "상황 종료"까지 홈에서 유지.
- 티칭 모드(관리자가 켬, 2026-10-07 사용자 요청): 사람이 옆에서 티칭하므로 사람 감지(감속·정지·사라짐)와
  인식 고장·어두움 판정을 하지 않는다. 작업(작업 순서) 실행은 막는다. 화재·비상정지·로봇 고장·장치 통신 고장은 그대로.
  켜면 사람·사라짐 때문에 걸린 정지를 풀고, 끄면 사라짐 추적을 처음부터 다시 한다(그 순간 사람이 가까우면 바로 정지).
"""
import logging
import threading
import time

import numpy as np

from .vanish import VanishTracker

log = logging.getLogger("safety")

FIRE, STOP, SLOW, CONVEYOR, NORMAL = "FIRE", "STOP", "SLOW", "CONVEYOR", "NORMAL"
STATE_KO = {FIRE: "화재", STOP: "정지", SLOW: "감속", CONVEYOR: "컨베이어 이상", NORMAL: "정상"}


class SafetyManager:
    def __init__(self, cfg, executor, perception, io, events):
        self.cfg = cfg
        self.ex = executor
        self.per = perception
        self.io = io
        self.events = events                    # EventHub: emit(type, msg, **kw)
        self.vanish = VanishTracker(cfg)
        self.lock = threading.RLock()
        self.state = STOP
        self.reasons = []
        self.stop_latched = "시스템 시작"   # 부팅 후 자동으로 움직이지 않는다
        self.human_alarm = False            # 사람이 정지 거리 안에 들어와 멈춤 → 재가동까지 화면 경보(붉은 깜빡임)
        self.zone = "CLEAR"
        self._zone_cand, self._zone_n = "CLEAR", 0
        self.min_dist = float("inf")
        self.maintenance = False
        self.teach_mode = False
        self.fire = False
        self.fire_event_id = None
        self.fire_homing = None
        self.conveyor_alarm = ""
        self.last_percept_seq = -1
        self.last_percept_t = time.monotonic()
        self.home_q_fn = lambda: None           # 홈 자세 (티칭 저장소에서)
        self.on_fire_start = []                 # 작업 중단 콜백 등
        self.guard_person_m = 0.0
        self.brightness = 255.0
        self._dark_t0 = None               # 무인 시험 보호: 보정과 무관하게 카메라 거리 이 안의 사람이면 정지
        self._run = True
        self.thread = threading.Thread(target=self._loop, name="safety", daemon=True)

    def start(self):
        self.thread.start()
        return self

    # ------------------------------------------------------------ 관리자 조작
    def restart(self, user="admin"):
        """재가동 승인. 조건을 다시 검사해 통과해야만 해제한다."""
        with self.lock:
            problems = self.restart_blockers()
            if problems:
                return False, problems
            prev = self.stop_latched
            self.stop_latched = ""
            self.human_alarm = False
            self.events.emit("restart", f"{user}이(가) 재가동 (해제한 정지: {prev})", level="info")
            return True, []

    def restart_blockers(self):
        p = []
        prof = self.profile()
        if self.fire:
            p.append("화재 상황 종료 전")
        if self.teach_mode:                     # 사람·인식 관련 조건은 보지 않는다
            if not self.io.healthy():
                p.append(f"장치 고장: {self.io.health_msg()}")
            if self.ex.fault:
                p.append(f"로봇 고장 — 먼저 고장 해제 ({self.ex.fault})")
            if self.ex.estop:
                p.append("비상정지 해제 필요")
            return p
        if self.vanish.unresolved:
            p.append(f"미확인 사라짐 {len(self.vanish.unresolved)}건 — 먼저 사라짐 확인")
        if self.vanish.pending:
            p.append("사라짐 판정 중")
        if self.min_dist < prof["clear"]:
            p.append(f"사람이 아직 가까움 ({self.min_dist:.2f} m, 기준 {prof['clear']:.2f} m)")
        if self._perception_stale():
            p.append("인식 멈춤 또는 카메라 고장")
        if self._dark_t0 is not None:
            p.append(f"조명이 어두움 (밝기 {self.brightness:.0f})")
        if not self.io.healthy():
            p.append(f"장치 고장: {self.io.health_msg()}")
        if self.ex.fault:
            p.append(f"로봇 고장 — 먼저 고장 해제 ({self.ex.fault})")
        if self.ex.estop:
            p.append("비상정지 해제 필요")
        return p

    def emergency_stop(self, source="비상정지 버튼"):
        """비상정지: 실행기를 즉시 세우고, 안전 관리자 정지도 같은 순간에 걸어 둔다(판단 주기를 기다리지 않음)."""
        self.ex.emergency_stop()
        with self.lock:
            self._latch(source, "estop")

    def clear_vanish(self, user="admin"):
        with self.lock:
            n = self.vanish.clear()
        self.events.emit("vanish_clear", f"{user}이(가) 사라짐 {n}건 확인 — 재가동은 따로 눌러야 함", level="info")
        return n

    def set_maintenance(self, on):
        with self.lock:
            self.maintenance = bool(on)
            if on:
                self.ex.stop_motion()
        self.events.emit("maintenance", "정비 모드 " + ("시작 — 로봇 동작 차단" if on else "종료"), level="info")

    HUMAN_LATCH = ("사람 근접", "사라짐", "인식 멈춤", "조명이 어두워")

    def set_teach_mode(self, on, user="admin"):
        on = bool(on)
        with self.lock:
            if on == self.teach_mode:
                return
            if on and self.fire:
                raise ValueError("화재 상황 중에는 티칭 모드를 켤 수 없음")
            self.teach_mode = on
            cleared = ""
            if on:
                n = self.vanish.clear()
                self.zone, self._zone_cand, self._zone_n = "CLEAR", "CLEAR", 0
                self.min_dist = float("inf")
                self.human_alarm = False
                self._dark_t0 = None
                if self.stop_latched.startswith(self.HUMAN_LATCH):
                    cleared, self.stop_latched = self.stop_latched, ""
                if n:
                    cleared += f"{' · ' if cleared else ''}미확인 사라짐 {n}건"
            else:
                self.vanish = VanishTracker(self.cfg)   # 티칭 중 보던 사람을 '사라짐'으로 오판하지 않게 처음부터
                self.last_percept_t = time.monotonic()
        if on:
            self.events.emit("teach_mode", f"{user}: 티칭 모드 시작 — 사람 감지로 멈추지 않음, 작업 실행 차단"
                             + (f" (해제: {cleared})" if cleared else ""), level="info")
        else:
            self.events.emit("teach_mode", f"{user}: 티칭 모드 종료 — 사람 감지 다시 적용", level="info")

    def end_fire(self, user="admin"):
        with self.lock:
            if not self.fire:
                return False
            self.fire = False
            self.stop_latched = "화재 상황 종료"
            eid = self.fire_event_id
            self.fire_event_id = None
        self.events.end_recording(eid)
        self.events.emit("fire_end", f"화재 상황 종료 ({user})", level="info")
        return True

    def profile(self):
        s = self.cfg["safety"]
        return s["profiles"][s["profile"]]

    # ------------------------------------------------------------ 판단 루프
    def _perception_stale(self):
        return (time.monotonic() - self.last_percept_t > self.cfg["perception"]["stale_timeout"]
                or bool(self.per.cam.error))

    def _loop(self):
        while self._run:
            try:
                self._step()
            except Exception:
                log.exception("safety step")          # 예외 시 허가를 보내지 않음 → 로봇 정지
                time.sleep(0.2)
                continue
            time.sleep(0.05)

    def _latch(self, why, etype, **kw):
        if not self.stop_latched:
            self.stop_latched = why
            self.events.emit(etype, why, level="alarm", clip=True, **kw)

    def _step(self):
        now = time.monotonic()
        with self.lock:
            prof = self.profile()
            s = self.cfg["safety"]
            # --- 인식 결과 반영 ---
            lat = self.per.latest
            if lat is not None and lat[1].seq != self.last_percept_seq:
                f, pc = lat
                self.last_percept_seq = pc.seq
                self.last_percept_t = now
                self.brightness = pc.brightness
                if not self.teach_mode:             # 티칭 모드: 사람 감지·어두움 판정 안 함
                    self._judge_people(pc, now, prof, s)
            if self._perception_stale() and not self.teach_mode:
                self._latch("인식 멈춤 또는 카메라 고장 — 고장 정지", "fault")
            if not self.io.healthy():
                self._latch(f"장치 통신 고장 — {self.io.health_msg()}", "fault")
            if self.ex.fault:
                self._latch(f"로봇 고장 — {self.ex.fault}", "fault")
            if self.ex.estop:
                self._latch("비상정지 버튼", "estop")

            # --- 화재 ---
            if self.io.fire() and not self.fire:
                self.fire = True
                self.fire_homing = None
                self.io.set_door(True)
                self.fire_event_id = self.events.emit("fire", "화재 감지 — 출입문 열림, 로봇 홈 복귀",
                                                      level="alarm", clip=True, until_end=True)
                for cb in self.on_fire_start:
                    cb()

            # --- 상태 결정 ---
            reasons = []
            if self.fire:
                state = FIRE
                reasons.append("화재 — 상황 종료까지 홈에서 대기")
            elif self.stop_latched or self.zone == "STOP":
                state = STOP
                reasons.append(self.stop_latched or "사람 근접")
            elif self.zone == "SLOW":
                state = SLOW
                reasons.append(f"사람 접근 ({self.min_dist:.2f} m)")
            elif self.conveyor_alarm:
                state = CONVEYOR
                reasons.append(self.conveyor_alarm)
            else:
                state = NORMAL
            if self.vanish.unresolved:
                reasons.append(f"미확인 사라짐 {len(self.vanish.unresolved)}건")
            if self.maintenance:
                reasons.append("정비 모드 (동작 차단)")
            if self.teach_mode:
                reasons.append("티칭 모드 — 사람 감지 안 함")
            if state != self.state:
                log.info("state %s -> %s (%s)", self.state, state, reasons)
            self.state, self.reasons = state, reasons

            # --- 허가 ---
            scale, why = self._permit_for(state, prof)
        self.ex.permit(scale, why)

    def _judge_people(self, pc, now, prof, s):
        """사람 감지 판정: 어두움, 사라짐, 감속·정지 거리. 티칭 모드에서는 부르지 않는다."""
        if pc.brightness < self.cfg["perception"].get("min_brightness", 30):
            self._dark_t0 = self._dark_t0 or now
            if now - self._dark_t0 > 1.5:
                self._latch(f"조명이 어두워 사람 인식 불가 — 고장 정지 (밝기 {pc.brightness:.0f})", "fault")
        else:
            self._dark_t0 = None
        new_v = self.vanish.update(pc, now)
        for v in new_v:
            self._latch_vanish(v)
        d = min(pc.min_dist, self.vanish.pending_min_dist())
        if self.guard_person_m > 0 and any(p.cam_dist < self.guard_person_m for p in pc.persons):
            d = 0.0
        self.min_dist = d
        cand = "STOP" if d < prof["stop"] else "SLOW" if d < prof["slow"] else (
            "CLEAR" if d > prof["clear"] else ("SLOW" if self.zone != "CLEAR" else "CLEAR"))
        if cand == self._zone_cand:
            self._zone_n += 1
        else:
            self._zone_cand, self._zone_n = cand, 1
        need = s["confirm_frames"] if cand != "CLEAR" else 3
        if self._zone_n >= need and cand != self.zone:
            prev, self.zone = self.zone, cand
            if cand == "SLOW":
                self.events.emit("human_slow", f"사람 접근으로 감속 ({d:.2f} m)", level="warn")
            if cand == "STOP":
                self.human_alarm = True       # 이미 다른 이유로 멈춰 있어도 사람 근접은 화면에 알린다
                self._latch(f"사람 근접으로 정지 ({d:.2f} m)", "human_stop")

    def _latch_vanish(self, v):
        self.stop_latched = self.stop_latched or "사라짐으로 정지 — 현장 확인 필요"
        self.events.emit("vanish", f"출구가 아닌 곳에서 사람이 사라짐 (마지막 거리 {v['dist']:.2f} m, "
                                   f"미확인 {len(self.vanish.unresolved)}건)", level="alarm", clip=True)

    def _permit_for(self, state, prof):
        near = self.zone == "STOP"
        if state == FIRE:
            # 화재: 사람이 가깝거나 미해결 사라짐이면 멈춤. 아니면 저속으로 홈 복귀만 허용.
            if near or self.vanish.unresolved or self.vanish.pending:
                return 0.0, "화재 — 사람 근접·사라짐으로 홈 복귀 보류"
            self._fire_home()
            return self.cfg["safety"]["slow_scale"], "화재 — 홈 복귀 중"
        if self.maintenance:
            return 0.0, "정비 모드"
        if state == STOP:
            return 0.0, self.reasons[0] if self.reasons else "정지"
        if state == SLOW:
            return self.cfg["safety"]["slow_scale"], "감속"
        return 1.0, STATE_KO[state]

    def _fire_home(self):
        """화재 중 홈 복귀 명령 (한 번만). 홈이 없으면 휴식 자세."""
        ex = self.ex
        if self.fire_homing is not None or not ex.torque_on or ex.fault or ex.estop:
            return
        q = self.home_q_fn()
        if q is None:
            q = list(ex.cfg["rest_pose"]) + [ex.current_q()[5]]
            q[0] = ex.current_q()[0]
        try:
            self.fire_homing = ex.move_joints(np.array(q, float), "fire_home")
        except Exception as e:
            self.fire_homing = False
            self.events.emit("fault", f"화재 홈 복귀 실패: {e}", level="alarm")

    def snapshot(self):
        with self.lock:
            return {
                "state": self.state,
                "state_ko": STATE_KO[self.state],
                "reasons": self.reasons,
                "stop_latched": self.stop_latched,
                "zone": self.zone,
                "min_dist": None if not np.isfinite(self.min_dist) else round(self.min_dist, 3),
                "profile": self.cfg["safety"]["profile"],
                "thresholds": self.profile(),
                "maintenance": self.maintenance,
                "teach_mode": self.teach_mode,
                "fire": self.fire,
                "human_alarm": self.human_alarm,
                "vanish_unresolved": len(self.vanish.unresolved),
                "vanish_pending": len(self.vanish.pending),
                "conveyor_alarm": self.conveyor_alarm,
                "restart_blockers": self.restart_blockers() if self.stop_latched else [],
                "guard_person_m": self.guard_person_m,
                "brightness": round(self.brightness),
            }
