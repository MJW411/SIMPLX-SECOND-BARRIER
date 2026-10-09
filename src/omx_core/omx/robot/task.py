"""반복 동작(자세 순서) 실행기와 컨베이어 감시.

- 순서의 각 단계는 실행기에 명령을 보내고 끝날 때까지 기다린다. 안전 정지로 허가가 끊기면
  실행기가 궤적 시간을 멈추므로 여기서는 그냥 계속 기다린다(그리퍼는 물체를 쥔 채 유지).
  재개되면 실행기가 다음 지점까지 저속으로 접근한다 — 단계를 건너뛰지 않는다.
- 부팅 시 자동으로 시작하지 않는다. 한 번 실행은 cycle_max_minutes 로 제한한다(무인 시험 한도).
"""
import logging
import threading
import time
from collections import deque

import numpy as np

from . import kinematics as K

log = logging.getLogger("task")


class _Paused(Exception):
    """작업 일시 정지 (컨베이어 이상 등) — 같은 단계를 재개 후 다시 한다."""


class CycleRunner:
    def __init__(self, ex, teach, io, events, limits, safety=None):
        self.ex, self.teach, self.io, self.events, self.L = ex, teach, io, events, limits
        self.safety = safety
        self.thread = None
        self.stop_evt = threading.Event()
        self.status = "대기"
        self.step_idx = -1
        self.cycles = 0
        self.grip_check = False          # 집기 실패 판정 (물체가 있을 때 켠다)
        self.t_start = 0.0
        self.max_cycles = 0
        self.done = False                # 정한 횟수를 다 마침 → 벽면 초록·완료 알림 (다음 시작·정지까지)
        # 일시 정지: 작업을 끝내지 않고 하던 단계에서 기다린다. 재개하면 그 단계부터 다시, 반복 횟수도 이어서.
        self.pause_evt = threading.Event()
        self.pause_why = ""
        self._pause_n = 0                # 일시 정지 횟수 — 멈췄다 바로 풀려도 그 사이 취소된 동작을 알아보려고
        self._sent_n = 0
        self.resume_pose = ""            # 재개할 때 먼저 가서 물품 도착을 기다릴 자세 이름 (빈 값 = 바로 이어서)
        # 정지 명령과 다음 동작 명령이 엇갈리지 않게: 동작을 보낼 때와 정지할 때 같은 잠금을 쓴다.
        # (예전: 정지 직후 단계 사이에 있던 실행 스레드가 다음 동작을 보내 로봇이 다시 움직일 수 있었다)
        self._cmd_lock = threading.Lock()

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def _check_start(self):
        if self.running:
            raise RuntimeError("이미 실행 중")
        seq = self.teach.snapshot()["sequence"]
        if not seq:
            raise RuntimeError("작업 순서가 비어 있음 — 자세 저장 후 작업 순서 작성 필요")
        if self.safety is not None and (self.safety.fire or self.safety.maintenance):
            raise RuntimeError("화재·정비 모드 중에는 시작 불가")
        if self.safety is not None and getattr(self.safety, "teach_mode", False):
            raise RuntimeError("티칭 모드 중에는 작업을 실행할 수 없음 — 티칭 모드를 끄고 실행")
        return seq

    def start(self, max_cycles=0, start_at=0):
        """반복 실행. start_at: 첫 회차만 이 단계부터 (선택한 단계부터 실행)."""
        seq = self._check_start()
        if not 0 <= start_at < len(seq):
            raise RuntimeError(f"없는 단계 {start_at + 1}")
        self.stop_evt.clear()
        self.pause_evt.clear()
        self.done = False
        self.max_cycles = int(max_cycles)
        self.thread = threading.Thread(target=self._run, args=(seq, max_cycles, start_at), daemon=True, name="cycle")
        self.thread.start()

    def run_one(self, index):
        """한 단계만 실행 (PolyScope 의 '여기로 이동'처럼 단계 시험용)."""
        seq = self._check_start()
        if not 0 <= index < len(seq):
            raise RuntimeError(f"없는 단계 {index + 1}")
        self.stop_evt.clear()
        self.pause_evt.clear()
        self.done = False

        def one():
            self.step_idx = index
            self.status = f"{index + 1}단계만 실행: {seq[index]['type']}"
            try:
                self._step_retry(seq[index], self.status)
                self.status = f"{index + 1}단계 실행 완료"
            except InterruptedError:
                self.status = "정지"
            except Exception as e:
                self.status = f"{index + 1}단계 실패: {e}"
            finally:
                self.step_idx = -1
        self.thread = threading.Thread(target=one, daemon=True, name="cycle-step")
        self.thread.start()

    def stop(self, why="정지 명령"):
        self.done = False
        if self.running:
            with self._cmd_lock:
                self.stop_evt.set()
                self.pause_evt.clear()
                self.ex.stop_motion()
            self.status = f"정지: {why}"

    @property
    def paused(self):
        return self.running and self.pause_evt.is_set()

    def pause(self, why="일시 정지", pose=""):
        """하던 동작만 멈추고(그 자리에서) 작업은 끝내지 않는다. resume() 하면 멈춘 단계부터 다시.
        pose 가 있으면 재개할 때 먼저 그 자세로 가서 물품이 도착 센서에 올 때까지 기다린 뒤 이어서 한다."""
        if not self.running or self.pause_evt.is_set():
            return False
        with self._cmd_lock:
            self.pause_why = why
            self.resume_pose = pose or ""
            self.pause_evt.set()
            self._pause_n += 1
            self.ex.stop_motion()
        self.status = f"일시 정지: {why}"
        log.info("작업 일시 정지 — %s", why)
        self.events.emit("cycle_pause", f"로봇 작업 일시 정지 — {why} (멈춘 자리에서 대기, {self.cycles}회 완료)", level="warn")
        return True

    def resume(self, why=""):
        if not self.pause_evt.is_set():
            return False
        self.pause_evt.clear()
        log.info("작업 재개%s", f" — {why}" if why else "")
        self.events.emit("cycle_resume", f"로봇 작업 재개{f' — {why}' if why else ''}"
                         f"{f' ({self.resume_pose} 자세에서 물품 도착 대기 후 이어서)' if self.resume_pose else ''}", level="info")
        return True

    def _wait_resume(self):
        while self.pause_evt.is_set():
            if self.stop_evt.is_set():
                raise InterruptedError()
            time.sleep(0.05)
        if self.stop_evt.is_set():
            raise InterruptedError()

    def _go_wait(self, name):
        """재개 직후: 자세 name 으로 가서(그리퍼는 그대로 — 쥔 물체 유지) 물품이 도착 센서에 올 때까지 기다린다.
        자세가 없거나 못 가면 기다리지 않고 바로 이어서 한다."""
        try:
            q = np.array(self.teach.pose_q(name), float)
        except Exception as e:
            log.warning("재개 대기 자세(%s) 없음: %s", name, e)
            return
        q[5] = self.ex.current_q()[5]
        self.status = f"재개: {name} 자세로 이동"
        try:
            self._wait(self._cmd(self.ex.move_joints, q, f"대기 {name}", speed_scale=0.5))
        except RuntimeError as e:                    # 동작 실패 (사람 근접 정지 등은 _wait 가 아닌 실행기에서 기다림)
            log.warning("재개 대기 자세(%s)로 못 감: %s", name, e)
            return
        if self.io is None:
            return
        self.status = f"재개: {name} 자세에서 물품 도착 대기"
        while not self.io.sensor2():
            if self.stop_evt.is_set():
                raise InterruptedError()
            if self.pause_evt.is_set():
                raise _Paused()
            time.sleep(0.05)

    def _step_retry(self, s, status):
        """단계를 실행한다. 일시 정지되면 그 자리에서 재개될 때까지 기다렸다가 같은 단계를 처음부터 다시 한다.
        재개 대기 자세가 정해져 있으면 (컨베이어 이상) 먼저 그 자세로 가서 물품 도착을 기다린다. 그 뒤
        다시 할 단계가 관절 이동(move·home)이 아니면 (직선 이동·그리퍼·센서 대기 등) 멈췄던 자리로 먼저 돌아간다
        — 예: 집으러 내려가던 중이었으면 그 높이로 돌아간 뒤 그리퍼를 닫는다."""
        back = None
        while True:
            try:
                if back is not None:
                    name, back_q = back
                    self._go_wait(name)
                    if s["type"] not in ("move", "home"):
                        q = np.array(back_q, float)
                        q[5] = self.ex.current_q()[5]
                        self._wait(self._cmd(self.ex.move_joints, q, "멈춘 자리로 복귀", speed_scale=0.5))
                    back = None
                    self.status = status
                self._step(s)
                return
            except _Paused:
                if back is None:
                    try:
                        back_q = self.ex.current_q()
                    except Exception:
                        back_q = None
                self._wait_resume()
                self.status = status
                name = self.resume_pose
                if name and back is None and back_q is not None:
                    back = (name, back_q)

    def _cmd(self, fn, *a, **k):
        """동작 명령을 보낸다. 이미 정지 명령이 왔으면 보내지 않는다."""
        with self._cmd_lock:
            if self.stop_evt.is_set():
                raise InterruptedError()
            if self.pause_evt.is_set():
                raise _Paused()
            self._sent_n = self._pause_n
            return fn(*a, **k)

    def _wait(self, m):
        while True:
            st = m.wait(0.1)
            if st != "running":
                if self.stop_evt.is_set():
                    raise InterruptedError()
                if st != "done" and (self.pause_evt.is_set() or self._sent_n != self._pause_n):
                    raise _Paused()                  # 일시 정지로 취소된 동작 (벌써 풀렸어도) → 재개 후 다시
                if st != "done":
                    raise RuntimeError(f"동작 {m.name} {'취소' if st == 'cancelled' else '실패'}: {m.reason}")
                return
            if self.stop_evt.is_set():
                raise InterruptedError()

    def _run(self, seq, max_cycles, start_at=0):
        self.t_start = time.monotonic()
        self.cycles = 0
        limit = self.L.get("cycle_max_minutes", 0) * 60       # 0 = 시간 제한 없음
        log.info("반복 시작 (%d단계)", len(seq))
        try:
            while not self.stop_evt.is_set():
                t_cycle = time.monotonic()
                for i, s in enumerate(seq):
                    if self.cycles == 0 and i < start_at:
                        continue
                    self.step_idx = i
                    self.status = f"{self.cycles + 1}회차 {i + 1}/{len(seq)}단계: {s['type']} {s.get('pose', s.get('value', ''))}"
                    self._step_retry(s, self.status)
                    if self.stop_evt.is_set():
                        raise InterruptedError()
                self.cycles += 1
                # 통계(대시보드 작업량·사이클 시간)용: 1회 걸린 시간(초). 이벤트 목록·알림에는 안 나온다
                self.events.emit("cycle_tick", f"{time.monotonic() - t_cycle:.2f}", level="info")
                if max_cycles and self.cycles >= max_cycles:
                    self.done = True
                    self.events.emit("cycle_done", f"작업 완료 — {self.cycles}회 반복", level="info")
                    break
                if limit and time.monotonic() - self.t_start > limit:
                    self.events.emit("cycle_stop", f"작업 자동 정지: 반복 시간 한도 {self.L['cycle_max_minutes']}분 도달", level="info")
                    break
            self.status = f"완료 ({self.cycles}회)"
        except InterruptedError:
            self.status = f"정지 ({self.cycles}회 완료)"
        except Exception as e:
            self.status = f"중단: {e}"
            self.events.emit("cycle_fail", f"반복 중단: {e}", level="warn")
        finally:
            self.step_idx = -1

    def _step(self, s):
        t = s["type"]
        ex = self.ex
        speed = s.get("speed", 100) / 100.0
        if t in ("move", "linear", "home"):
            # 팔만 움직인다. 그리퍼는 그리퍼 단계로만 (쥔 물체 유지)
            if t == "home":
                hq = self.teach.home_q()
                if hq is None:
                    raise RuntimeError("홈 자세가 지정되지 않음 — 티칭에서 '홈으로 지정'")
                q, name = np.array(hq, float), "home"
            else:
                q, name = np.array(self.teach.pose_q(s["pose"]), float), s["pose"]
            q[5] = ex.current_q()[5]
            if t == "linear":
                x, y, z, p = K.fk(q)
                self._wait(self._cmd(ex.move_linear, x, y, z, p, q5=q[4], name=f"linear {name}", speed_scale=speed))
            else:
                self._wait(self._cmd(ex.move_joints, q, f"{t} {name}", speed_scale=speed))
        elif t == "grip":
            g = self.teach.snapshot()["gripper"]
            val = s.get("value", "open")
            target, force, what = self.teach.grip_action(s)
            ex.set_gripper_force(force)
            self._wait(self._cmd(ex.move_gripper, target, f"grip {what}"))
            time.sleep(0.4)
            if val == "close" and not s.get("preset") and self.grip_check:
                pos = ex.state.q[5]
                if abs(pos - g["close"]) < 0.06:        # 끝까지 닫힘 = 잡은 물체 없음 (열림·닫힘 값 방향과 무관)
                    self.events.emit("pick_fail", f"집기 실패 — 그리퍼가 끝까지 닫힘 ({pos:.2f} rad)", level="warn")
                    raise RuntimeError("집기 실패")
        elif t == "wait_sensor2":
            # 컨베이어는 ConveyorMonitor 가 움직인다(작업 중 + 도착 센서 비면 가동, 감지되면 정지). 여기서는 도착만 기다린다.
            while not self.io.sensor2():
                if self.stop_evt.is_set():
                    raise InterruptedError()
                if self.pause_evt.is_set():
                    raise _Paused()
                time.sleep(0.05)
        elif t == "conveyor":
            self._cmd(self.io.set_conveyor, s.get("value", "on") == "on")
        elif t == "pause":
            end = time.monotonic() + float(s.get("value", 1.0))
            while time.monotonic() < end:
                if self.stop_evt.is_set():
                    raise InterruptedError()
                if self.pause_evt.is_set():
                    raise _Paused()
                time.sleep(0.05)

    def snapshot(self):
        return {"running": self.running, "status": self.status, "step": self.step_idx, "cycles": self.cycles,
                "max_cycles": self.max_cycles, "done": self.done, "grip_check": self.grip_check,
                "paused": self.paused, "pause_why": self.pause_why if self.paused else "",
                "elapsed": round(time.monotonic() - self.t_start) if self.running else 0}


def pickplace_template(poses):
    """자세 이름 규칙(home, pick_above, pick, place_above, place)으로 픽앤플레이스 순서 생성."""
    need = ["pick_above", "pick", "place_above", "place"]
    miss = [n for n in need if n not in poses]
    if miss:
        raise ValueError("자세 저장 필요: " + ", ".join(miss))
    return [
        {"type": "home"},
        {"type": "grip", "value": "open"},
        {"type": "wait_sensor2"},
        {"type": "move", "pose": "pick_above"},
        {"type": "linear", "pose": "pick"},
        {"type": "grip", "value": "close"},
        {"type": "linear", "pose": "pick_above"},
        {"type": "move", "pose": "place_above"},
        {"type": "linear", "pose": "place"},
        {"type": "grip", "value": "open"},
        {"type": "linear", "pose": "place_above"},
        {"type": "home"},
    ]


class ConveyorMonitor:
    """컨베이어 제어와 감시 (2026-10-08: 센서 둘 — 투입 D11 = sensor1, 도착 D12 = sensor2).

    - 작업(작업 순서)이 실행 중이고 투입 센서에 물품이 올라오면 출발한다 (빈 컨베이어는 돌리지 않는다).
      출발한 뒤에는 물품이 투입 센서를 지나가도 도착할 때까지 계속 돈다.
      conveyor.start_sensor: false 면 예전처럼 작업 중 도착 센서가 비어 있는 동안 계속 돌린다.
    - 도착 센서가 감지하거나 작업이 끝나면 멈춘다 (아두이노도 도착 센서 감지 시 스스로 즉시 멈춘다).
    - 한 번 돌기 시작해 제한시간(travel_timeout, 0=끔) 안에 도착 센서가 감지하지 않으면 멈추고
      컨베이어 이상(물품 미도달): 이벤트·녹화·알림. 이상인 동안은 다시 돌리지 않는다.
    - 이상이 나면 로봇 작업(작업 순서)을 일시 정지한다 (끝내지 않고 하던 단계에서 기다림).
    - 자동 해제 + 자동 재개 (2026-10-09 사용자): 투입 센서에 물품이 새로 올라오거나(없다가 생김), 물품이 도착 센서에
      닿거나, 작업을 새로 시작하거나, 관리자가 해제 버튼을 누르면 이상을 풀고 로봇 작업을 멈춘 단계부터 이어서 한다.
      이상이 걸린 순간 투입 센서에 이미 물품이 있었다면 한 번 치웠다 다시 올려야 한다 (막힌 채 저절로 다시 돌지 않게).
      관리자 '해제' 버튼도 그대로 쓸 수 있다.
    - 화재면 멈춘다.
    """

    def __init__(self, io, events, cfg, safety, runner=None, start=True):
        self.io, self.events, self.cfg, self.safety, self.runner = io, events, cfg, safety, runner
        self.on = False
        self.t_run = None
        self.last_travel = None
        self.history = deque(maxlen=20)              # 최근 도착 기록 (제어판 차트): 걸린 시간, 이상 여부
        # 이 감시기가 건 이상 / 푼 시각 — 안전 노드 상태가 늦게 와도(노드 분리) 바로 반영되게
        self._jam = False
        self._jam_seen = False
        self._clear_t = -1e9
        self._was_working = False
        self._prev_loaded = False
        self._paused_by_jam = False
        if start:
            threading.Thread(target=self._loop, daemon=True, name="conveyor").start()

    def warning(self):
        to = self.cfg["conveyor"]["travel_timeout"]
        pre = self.cfg["recording"]["pre_sec"]
        if to > pre:
            return f"제한시간 {to:.0f}s가 사전 녹화 {pre}s보다 김 — 클립에 컨베이어가 돌기 시작한 장면이 빠질 수 있음"
        return ""

    def _loop(self):
        while True:
            try:
                self.step(time.monotonic())
            except Exception:
                log.exception("conveyor")
            time.sleep(0.05)

    def _set(self, on, why):
        self.on = on
        self.io.set_conveyor(on)
        log.info("컨베이어 %s — %s", "가동" if on else "정지", why)

    def step(self, now):
        if not self.io.ready():
            return
        use_arrive = self.cfg["conveyor"].get("arrive_sensor", True)
        if hasattr(self.io, "set_arrive_stop"):
            self.io.set_arrive_stop(use_arrive)
        arrived = self.io.sensor2() if use_arrive else False     # 도착 센서 고장 시 임시: 작업 중이면 계속 돈다
        fire = self.io.fire() or self.safety.fire
        working = self.runner is not None and self.runner.running
        use_start = self.cfg["conveyor"].get("start_sensor", True)
        loaded = self.io.sensor1() if use_start else True
        new_item = use_start and loaded and not self._prev_loaded       # 투입 센서: 없다가 생긴 순간
        self._prev_loaded = loaded if use_start else False
        alarm = self._alarm(now)
        if alarm and (arrived or new_item or (working and not self._was_working)):
            self._clear(now, "물품이 도착 센서에 닿음" if arrived else "투입 센서에 새 물품" if new_item else "작업을 새로 시작")
            alarm = False
        if not alarm and self._paused_by_jam:          # 어떤 방법으로든 풀렸으면 로봇 작업 재개 (관리자 해제 포함)
            self._paused_by_jam = False
            if hasattr(self.runner, "resume"):
                self.runner.resume("컨베이어 이상 해제")
        self._was_working = working
        ok = working and not arrived and not fire and not alarm
        want = ok if self.on else (ok and loaded)      # 출발은 투입 센서에 물품이 있을 때, 계속 도는 건 도착까지
        if want and not self.on:
            self.t_run = now
            self._set(True, "작업 중, 투입 센서에 물품" if use_start else "작업 중, 도착 센서 비어 있음")
        elif not want and self.on:
            if arrived:
                self.last_travel = now - self.t_run
                self.history.append({"t": round(time.time()), "s": round(self.last_travel, 2), "jam": False})
                why = f"도착 센서 감지 ({self.last_travel:.1f}초)"
            else:
                why = "화재" if fire else "작업 끝" if not working else "컨베이어 이상"
            self._set(False, why)
            self.t_run = None
        to = self.cfg["conveyor"]["travel_timeout"]
        if self.on and to > 0 and now - self.t_run > to:
            self._set(False, "제한시간 초과")
            self.t_run = None
            msg = f"컨베이어 이상 — {to:.0f}초 동안 돌았지만 도착 센서에 물품이 오지 않음, 컨베이어 정지"
            self._jam, self._jam_seen = True, False
            self.history.append({"t": round(time.time()), "s": round(float(to), 2), "jam": True})
            self.safety.conveyor_alarm = msg
            self.events.emit("conveyor_jam", msg, level="alarm", clip=True)
            # 컨베이어가 비정상으로 멈추면 로봇 작업은 일시 정지 (2026-10-09 사용자: 해제되면 자동으로 이어서)
            if self.runner is not None and hasattr(self.runner, "pause"):
                self.runner.pause("컨베이어 이상", self.cfg["conveyor"].get("resume_pose", ""))
                self._paused_by_jam = True
            elif self.runner is not None and hasattr(self.runner, "stop"):
                self.runner.stop("컨베이어 이상")

    def _alarm(self, now):
        a = bool(self.safety.conveyor_alarm)
        if now - self._clear_t < 1.0:                 # 방금 풀었다: 안전 노드 상태가 아직 옛 값일 수 있다
            a = False
        if self._jam:
            if a:
                self._jam_seen = True
            elif self._jam_seen:                       # 관리자가 해제 버튼으로 풀었다
                self._jam = False
        return a or self._jam

    def _clear(self, now, why):
        self._jam = self._jam_seen = False
        self._clear_t = now
        self.safety.conveyor_alarm = ""
        log.info("컨베이어 이상 자동 해제 — %s", why)
        self.events.emit("conveyor_clear", f"컨베이어 이상 자동 해제 — {why}", level="info")

    def snapshot(self):
        return {"timing": self.on, "last_travel": self.last_travel, "history": list(self.history),
                "running_for": round(time.monotonic() - self.t_run, 1) if self.on and self.t_run is not None else None,
                "timeout": self.cfg["conveyor"]["travel_timeout"], "warning": self.warning()}
