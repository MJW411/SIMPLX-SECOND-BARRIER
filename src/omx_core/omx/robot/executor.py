"""실행기 — 로봇에 명령을 보내는 유일한 통로.

원칙
- 안전 관리자의 "움직여도 됨" 허가(permit)가 permit_timeout 안에 계속 들어와야 움직인다.
  허가가 끊기거나 배율이 0이면 궤적 시간이 멈춰 그 자리에 선다.
- 목표는 50Hz로 현재 위치 근처의 작은 점만 보낸다. 프로그램이 죽어도 모터는 마지막
  작은 목표에서 멈춘다(자리 유지).
- 모든 목표점은 관절 한계(여유 포함)·작업 상자·링크 최소 높이 검사를 통과해야 한다.
- 온도·통신·위치 오차·부하·하드웨어 오류를 매 주기 감시하고, 이상이면 고장으로 멈춘다.
"""
import logging
import math
import threading
import time

import numpy as np

from . import kinematics as K
from .dxl_bus import BusError

log = logging.getLogger("executor")
JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
HW_ERR_BITS = {0: "입력 전압", 2: "과열", 3: "엔코더", 4: "전기 충격", 5: "과부하"}


class LimitError(ValueError):
    pass


class Motion:
    """관절 경유점 경로 + 사다리꼴 시간 배정. 시간 배율로 멈춤·감속한다."""

    def __init__(self, waypoints, seg_times, ta, name=""):
        self.wp = np.asarray(waypoints, float)
        self.cum = np.concatenate([[0.0], np.cumsum(seg_times)])
        self.T = float(self.cum[-1])
        self.ta = max(1e-3, min(ta, self.T)) if self.T > 0 else 1e-3
        self.t = 0.0
        self.name = name
        self.status = "running"          # running | done | cancelled | failed
        self.reason = ""
        self.done_evt = threading.Event()

    @property
    def total(self):
        return self.T + self.ta

    def _u(self, t):
        T, ta = self.T, self.ta
        if t <= ta:
            return t * t / (2 * ta)
        if t <= T:
            return t - ta / 2
        return T - (T + ta - t) ** 2 / (2 * ta)

    def sample(self, t):
        if self.T <= 0:
            return self.wp[-1]
        u = min(max(self._u(min(t, self.total)), 0.0), self.T)
        k = int(np.searchsorted(self.cum, u, side="right") - 1)
        k = min(max(k, 0), len(self.wp) - 2)
        seg = self.cum[k + 1] - self.cum[k]
        a = 0.0 if seg <= 0 else (u - self.cum[k]) / seg
        return self.wp[k] + (self.wp[k + 1] - self.wp[k]) * a

    def finish(self, status, reason=""):
        if self.status == "running":
            self.status = status
            self.reason = reason
            self.done_evt.set()

    def wait(self, timeout=None):
        self.done_evt.wait(timeout)
        return self.status


class Executor:
    def __init__(self, bus, cfg, limits, on_event=None):
        self.bus = bus
        self.cfg = cfg
        self.L = limits
        self.on_event = on_event or (lambda *a, **k: None)
        self.dt = 1.0 / cfg["control_hz"]
        lim = limits["joint_limits"]
        m = limits["joint_margin"]
        self.qmin = np.array([lim[j][0] + m for j in JOINTS] + [limits["gripper_limits"][0]])
        self.qmax = np.array([lim[j][1] - m for j in JOINTS] + [limits["gripper_limits"][1]])
        self.box = limits["workspace_box"]
        self.speed_ratio = limits["speed_default_ratio"]
        self.cart_speed = limits["cart_speed_default"]

        self.lock = threading.RLock()
        self.goal = None                 # 마지막으로 보낸 목표 (6)
        self.motion = None
        self.jog = None                  # (vec, t_last)  좌표: dx,dy,dz,dpitch,dgrip / 관절: d1..d5,dgrip
        self.jog_mode = "cart"
        self.jog_target = None           # (x,y,z,pitch,q5,grip)
        self.torque_on = False
        self.freedrive = False           # 팔 관절 토크를 끄고 손으로 옮기는 모드 (그리퍼는 유지)
        self.estop = False
        self.fault = ""                  # 비어있지 않으면 고장 정지
        self.permit_scale = 0.0
        self.permit_reason = "시작 전"
        self.permit_t = 0.0
        self.slow_after_stop = False
        self.scale_now = 0.0
        self.comm_fail = 0
        self.write_fail = 0              # 목표 쓰기 연속 실패 (읽기 성공이 이 값을 지우지 않게 따로 센다)
        self.track_t0 = None
        self.load_t0 = None
        self.stall_t0 = np.full(5, np.nan)
        self.stall_note = ""
        self.stall_logged = np.full(5, -1e9)          # 관절별 마지막 막힘 기록 시각 (로그 폭주 방지)
        self.temp_warned = False
        self.angle_warned = np.zeros(5, bool)
        self.jlo = np.array([lim[j][0] for j in JOINTS])   # 기구 한계 (여유 제외)
        self.jhi = np.array([lim[j][1] for j in JOINTS])
        self.last_hw_check = 0.0
        self.jog_blocked = ""
        self.state = None
        self.loop_ms = 0.0
        self._run = True
        self.thread = threading.Thread(target=self._loop, name="executor", daemon=True)

    # ------------------------------------------------------------------ 공개 API
    def start(self):
        self.bus.configure(self.cfg["profile_time_ms"], self.cfg["profile_acc_ms"],
                           self.L["arm_pwm_ratio"], self.cfg.get("bus_watchdog_ms", 0), self._pwm_per_joint(),
                           self.cfg.get("gripper_gains"))
        self.state = self.bus.read_state()
        self.torque_on = all(self.bus.torque_state())
        if self.torque_on:
            self.goal = self.state.q.copy()
        self.thread.start()

    def _pwm_per_joint(self):
        pj = self.L.get("arm_pwm_ratio_joints", {}) or {}
        return {JOINTS.index(k): float(v) for k, v in pj.items() if k in JOINTS}

    def shutdown(self):
        self._run = False
        self.thread.join(timeout=1.0)

    def permit(self, scale, reason=""):
        """안전 관리자만 부른다. 주기적으로 계속 불려야 움직인다."""
        with self.lock:
            self.permit_scale = float(max(0.0, min(1.0, scale)))
            self.permit_reason = reason
            self.permit_t = time.monotonic()

    def set_speed(self, ratio):
        with self.lock:
            self.speed_ratio = float(min(max(ratio, 0.01), self.L["speed_cap_ratio"]))
        return self.speed_ratio

    def _goal_here(self):
        """지금 자세를 목표로. 단 그리퍼는 직전 목표를 유지한다 — 전류 기반 그리퍼는 목표가 실제 위치(물체에 막힌 곳)와
        같아지면 조이는 힘이 사라져 쥔 물체가 미끄러진다 (복구·토크 켜기·직접 교시 끝날 때)."""
        g = self.state.q.copy()
        if self.goal is not None:
            g[5] = self.goal[5]
        return g

    def set_torque(self, on, force=False):
        with self.lock:
            if on:
                if self.fault:
                    raise LimitError(f"로봇 고장: {self.fault}")
                self.bus.torque(True)
                self.state = self.bus.read_state()
                self.goal = self._goal_here()
                self.torque_on = True
                self.on_event("robot", "토크 켜짐")
            else:
                if not force and not self.at_rest() and not self.freedrive:
                    raise LimitError("휴식 자세가 아니면 토크를 끌 수 없음 (팔 낙하 위험) — "
                                     "휴식 후 토크 끄기 또는 강제 끄기 사용")
                self._cancel("토크 꺼짐")
                self.bus.torque(False)
                self.torque_on = False
                self.freedrive = False
                self.goal = None
                self.on_event("robot", "토크 꺼짐")

    def set_freedrive(self, on):
        """프리드라이브: 팔 관절 5개 토크를 끄고(그리퍼는 계속 쥠) 손으로 옮긴다. 끌 때는 현재 위치를 목표로 토크를 켠다."""
        with self.lock:
            if on:
                if self.freedrive:
                    return
                if not self.torque_on:
                    raise LimitError("먼저 토크 켜기 필요 (그리퍼를 쥔 채 유지)")
                if self.estop or self.fault:
                    raise LimitError("비상정지 또는 고장 상태")
                self._cancel("직접 교시")
                self.jog = None
                self.bus.torque_subset(range(5), False)
                self.freedrive = True
                self.torque_on = False          # 움직임 명령 거부
                log.info("freedrive on")
                self.on_event("robot", "직접 교시 시작 — 팔 토크 꺼짐, 손으로 이동")
            else:
                if not self.freedrive:
                    return
                self.bus.torque_subset(range(5), True)
                self.state = self.bus.read_state()
                self.goal = self._goal_here()
                self.freedrive = False
                self.torque_on = True
                log.info("freedrive off at %s", np.round(self.state.q, 3).tolist())
                self.on_event("robot", "직접 교시 종료 — 현재 자세에서 토크 켜짐")

    def emergency_stop(self):
        with self.lock:
            self.estop = True
            self._cancel("비상정지")
            self.jog = None
        self.on_event("estop", "비상정지 버튼")

    def reset(self):
        """비상정지·고장 해제 (관리자). 움직임은 다시 명령해야 한다.
        - 통신 고장이면 포트를 다시 연다 (전원 재투입으로 ttyACM 번호가 바뀌어도 찾는다)
        - 하드웨어 오류(과부하 등)가 남은 모터는 재부팅해 오류를 지운다. 그 모터는 이미 스스로 토크를 끈 상태다.
        재연결·재부팅 뒤에는 토크를 '꺼짐'으로 보고, 관리자가 '토크 켜기'를 눌러야 한다."""
        msgs = []
        with self.lock:
            if self.state is not None and self.state.temp.max() > self.L["temp_resume_c"]:
                raise LimitError(f"모터가 아직 뜨거움 ({self.state.temp.max():.0f}°C)")
            need_reconnect = self.fault.startswith("통신 오류") or self.comm_fail > 0
        if need_reconnect:
            try:
                self.bus.reconnect()
                msgs.append(f"재연결 완료 ({getattr(self.bus, 'port_name', '')})")
            except Exception as e:
                raise LimitError(f"재연결 실패: {e} — USB·전원 확인")
        try:
            errs = self.bus.read_hw_errors()
        except BusError as e:
            raise LimitError(f"모터 응답 없음: {e}")
        for k in [int(i) for i in np.nonzero(errs)[0]]:
            self.bus.reboot(k)
            msgs.append(f"ID{self.bus.ids[k]} 재부팅 (오류 해제)")
        with self.lock:
            self.state = self.bus.read_state()
            tq = self.bus.torque_state()
            self.estop = False
            self.fault = ""
            self.comm_fail = 0
            self.write_fail = 0
            self._cancel("고장 해제")
            self.jog = None
            if msgs or not all(tq):
                self.torque_on = False                  # 관리자가 다시 토크를 켠다
                self.goal = self.state.q.copy() if any(tq) else None
            elif self.torque_on:
                self.goal = self._goal_here()
        if msgs:
            self.on_event("robot", "고장 해제: " + ", ".join(msgs) + " — 토크 다시 켜기 필요")
        return ", ".join(msgs) or "해제 완료"

    def stop_motion(self):
        with self.lock:
            self._cancel("정지 명령")
            self.jog = None

    def at_rest(self, tol=0.15):
        if self.state is None:
            return False
        d = np.abs(self.state.q[1:5] - np.array(self.cfg["rest_pose"][1:]))   # 베이스 회전은 무관
        return bool(d.max() < tol)

    def current_q(self):
        with self.lock:
            q = (self.goal if self.goal is not None else self.state.q).copy()
            q[5] = min(max(q[5], self.qmin[5]), self.qmax[5])   # 그리퍼는 한계 안으로
            return q

    # ---- 검사 ----
    def check_q(self, q, q_from=None):
        """q_from 이 주어지면, 이미 한계 밖에 있던 관절은 한계 쪽으로 돌아오는(나빠지지 않는) 움직임을 허용한다
        (한계를 좁힌 뒤에도 바깥에 있는 로봇을 안으로 되돌릴 수 있게)."""
        q = np.asarray(q, float)

        def viol(v, i):
            return max(self.qmin[i] - v, v - self.qmax[i], 0.0)
        bad = [f"{(JOINTS + ['gripper'])[i]}={q[i]:.2f}" for i in range(6)
               if viol(q[i], i) > 1e-6 and (q_from is None or viol(q[i], i) > viol(q_from[i], i) + 1e-6)]
        if bad:
            raise LimitError("관절 한계 초과: " + ", ".join(bad))
        pts = K.fk_points(q)
        x, y, z = pts[-1]
        b = self.box
        if not (b["x"][0] <= x <= b["x"][1] and b["y"][0] <= y <= b["y"][1] and b["z"][0] <= z <= b["z"][1]):
            raise LimitError(f"작업 영역 밖: TCP=({x:.3f},{y:.3f},{z:.3f})")
        if pts[3:, 2].min() < self.L["min_link_z"]:
            raise LimitError(f"링크 위치가 너무 낮음 (z={pts[3:, 2].min():.3f})")

    def solve(self, x, y, z, pitch, q5=0.0, grip=None, q_ref=None):
        """TCP 목표 → 관절 (검사 포함). 팔꿈치 위 해가 한계 밖이면 오류."""
        q_ref = self.current_q() if q_ref is None else q_ref
        qa = K.ik(x, y, z, pitch, q5, q1_hint=q_ref[0])
        q = np.concatenate([qa, [q_ref[5] if grip is None else grip]])
        self.check_q(q)
        return q

    # ---- 움직임 명령 ----
    def _vlimit(self, scale=1.0):
        r = self.speed_ratio * min(1.0, max(0.05, scale))      # scale: 단계별 속도 (현재 속도 설정의 비율)
        return r * self.L["joint_vel_max"], max(r, 0.1) * self.L["joint_acc_max"]

    def _require_ready(self):
        if self.freedrive:
            raise LimitError("직접 교시 중 — 먼저 직접 교시 종료")
        if not self.torque_on:
            raise LimitError("토크 꺼짐")
        if self.estop:
            raise LimitError("비상정지 중")
        if self.fault:
            raise LimitError(f"로봇 고장: {self.fault}")

    def move_joints(self, q_target, name="move", speed_scale=1.0):
        with self.lock:
            self._require_ready()
            q0 = self.current_q()
            q1 = np.asarray(q_target, float).copy()
            self.check_q(q1)
            # 관절 보간 경로 중간점도 검사 (팔이 작업대를 스치지 않게)
            n = max(2, int(np.abs(q1 - q0).max() / 0.05) + 1)
            for a in np.linspace(0, 1, n)[1:-1]:
                self.check_q(q0 + (q1 - q0) * a, q_from=q0)
            vmax, amax = self._vlimit(speed_scale)
            seg = float(np.abs(q1 - q0).max() / vmax)
            return self._start(Motion([q0, q1], [seg], vmax / amax, name))

    def move_linear(self, x, y, z, pitch, q5=None, grip=None, name="linear", speed_scale=1.0):
        """TCP 직선 이동 (집기·놓기 접근용). 경로 전체를 미리 IK·검사한다."""
        with self.lock:
            self._require_ready()
            q0 = self.current_q()
            p0 = K.fk(q0)
            q5 = q0[4] if q5 is None else q5
            g1 = q0[5] if grip is None else grip
            p1 = np.array([x, y, z, pitch])
            dist = float(np.linalg.norm(p1[:3] - p0[:3]))
            n = max(2, int(max(dist / 0.005, abs(p1[3] - p0[3]) / 0.05)) + 1)
            wps = []
            for a in np.linspace(0, 1, n):
                p = p0 + (p1 - p0) * a
                q = self.solve(*p, q5=q0[4] + (q5 - q0[4]) * a,
                               grip=q0[5] + (g1 - q0[5]) * a, q_ref=wps[-1] if wps else q0)
                wps.append(q)
            wps[0] = q0
            for k in range(len(wps) - 1):
                if np.abs(wps[k + 1][:5] - wps[k][:5]).max() > 0.15:
                    raise LimitError("직선 경로 중 관절 급변 (특이점·IK 해 전환) — 먼저 관절 이동 필요")
            vmax, amax = self._vlimit(speed_scale)
            vcart = self.cart_speed * self.speed_ratio * min(1.0, max(0.05, speed_scale)) / self.L["speed_default_ratio"]
            vcart = min(vcart, self.L["cart_speed_max"])
            seg = []
            for k in range(len(wps) - 1):
                dq = float(np.abs(wps[k + 1] - wps[k]).max()) / vmax
                dp = float(np.linalg.norm(K.fk_points(wps[k + 1])[-1] - K.fk_points(wps[k])[-1])) / vcart
                seg.append(max(dq, dp, 1e-3))
            return self._start(Motion(wps, seg, vmax / amax, name))

    def align(self, target="down"):
        """TCP 위치는 그대로, 그리퍼 방향만 정렬: down = Z축(수직 아래, 기울기 90°), horizontal = 수평(0°)."""
        pitch = {"down": math.pi / 2, "horizontal": 0.0}[target]
        q = self.current_q()
        x, y, z, _ = K.fk(q)
        try:
            return self.move_linear(x, y, z, pitch, name=f"align_{target}")
        except (K.IKError, LimitError) as e:
            raise LimitError(f"현재 위치에서 {'Z축 수직 정렬' if target == 'down' else '수평 정렬'} 불가 ({e}) — "
                             "TCP를 조금 옮긴 뒤 (위·앞 등) 다시 시도")

    def move_gripper(self, g, name="gripper"):
        q = self.current_q()
        q[5] = g
        return self.move_joints(q, name)

    def _start(self, m):
        log.info("motion start: %s → %s", m.name, np.round(m.wp[-1], 3).tolist())
        self._cancel("새 명령")
        self.jog = None
        self.motion = m
        return m

    def _cancel(self, reason):
        if self.motion is not None:
            self.motion.finish("cancelled", reason)
            self.motion = None
        self.slow_after_stop = False

    def jog_cmd(self, vec, mode="cart"):
        """조그 (키를 누르는 동안 반복 호출). 0.25초 안에 다음 호출이 없으면 멈춘다(데드맨).
        mode="cart": vec = [x,y,z,pitch,grip] 방향 (-1,0,1)
        mode="joint": vec = [관절1..5, 그리퍼] 방향 (-1,0,1) — 관절을 하나씩 직접 돌린다."""
        mode = "joint" if mode == "joint" else "cart"
        vec = np.asarray(vec, float)
        if len(vec) != (6 if mode == "joint" else 5):
            raise LimitError("조그 명령 형식 오류")
        with self.lock:
            self._require_ready()
            if self.motion is not None:
                self._cancel("조그")
            if self.jog is None or self.jog_target is None or mode != self.jog_mode:
                log.info("jog start (%s): %s", mode, list(vec))
                q = self.current_q()
                self.jog_target = np.concatenate([K.fk(q), [q[4], q[5]]])
            self.jog_mode = mode
            self.jog = (np.clip(vec, -1, 1), time.monotonic())

    def set_gripper_force(self, pct):
        """그리퍼 힘 (전류 한도의 %, 5~100). 100% = gripper_current_cap_ma."""
        cap = self.L.get("gripper_current_cap_ma", 500)
        ma = int(round(cap * min(100.0, max(5.0, float(pct))) / 100.0))
        if getattr(self.bus, "gripper_current", None) != ma:
            self.bus.set_gripper_current(ma)
        return ma

    def gripper_force_pct(self):
        cap = self.L.get("gripper_current_cap_ma", 500)
        return round(100.0 * getattr(self.bus, "gripper_current", self.L["gripper_current_ma"]) / cap)

    # ------------------------------------------------------------------ 제어 루프
    def _loop(self):
        next_t = time.monotonic()
        while self._run:
            t0 = time.monotonic()
            try:
                self._tick(t0)
            except Exception as e:                       # 루프는 죽으면 안 된다
                log.exception("executor tick")
                self._set_fault(f"실행기 오류: {e}")
            self.loop_ms = (time.monotonic() - t0) * 1000
            next_t += self.dt
            sl = next_t - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            else:
                next_t = time.monotonic()

    def _set_fault(self, why):
        with self.lock:
            if not self.fault:
                self.fault = why
                self._cancel(why)
                self.jog = None
                log.warning("FAULT: %s", why)
                self.on_event("fault", why)

    def _tick(self, now):
        # 1) 상태 읽기
        try:
            st = self.bus.read_state()
            self.comm_fail = 0
        except BusError as e:
            self.comm_fail += 1
            if self.comm_fail >= self.L["comm_fail_max"]:
                self._set_fault(f"통신 오류: {e}")
            return
        self.state = st
        if now - self.last_hw_check > 1.0:
            self.last_hw_check = now
            try:
                errs = self.bus.read_hw_errors()
                if errs.any():
                    names = [f"ID{self.bus.ids[i]}:" + ",".join(v for b, v in HW_ERR_BITS.items() if errs[i] >> b & 1)
                             for i in np.nonzero(errs)[0]]
                    self._set_fault("하드웨어 오류 " + " ".join(names))
            except BusError:
                pass

        with self.lock:
            # 2) 감시
            if st.temp.max() >= self.L["temp_stop_c"]:
                self._set_fault(f"모터 과열로 정지 ({st.temp.max():.0f}°C) — 식을 때까지 대기")
            tw = self.L.get("temp_warn_c", self.L["temp_stop_c"] - 5)
            if not self.temp_warned and st.temp.max() >= tw:
                self.temp_warned = True
                i = int(np.argmax(st.temp))
                self.on_event("temp", f"모터 온도 높음: ID{self.bus.ids[i]} {st.temp[i]:.0f}°C "
                                      f"(경고 {tw}°C, {self.L['temp_stop_c']}°C에서 정지)")
            elif self.temp_warned and st.temp.max() < tw - 3:
                self.temp_warned = False
            out = (st.q[:5] < self.jlo - 0.05) | (st.q[:5] > self.jhi + 0.05)
            for j in np.nonzero(out & ~self.angle_warned)[0]:
                self.on_event("angle", f"{JOINTS[j]} 각도가 기구 한계 밖 ({st.q[j]:.2f} rad, 범위 "
                                       f"{self.jlo[j]:.2f}~{self.jhi[j]:.2f}) — 외력 또는 보정 이상 의심")
            self.angle_warned = out
            if not self.torque_on or self.goal is None:
                return
            err = float(np.abs(self.goal[:5] - st.q[:5]).max())
            if err > self.L["track_error_rad"]:
                self.track_t0 = self.track_t0 or now
                if now - self.track_t0 > self.L["track_error_time"]:
                    self._set_fault(f"위치 오차 과다 ({err:.2f} rad) — 충돌 또는 과부하 의심")
            else:
                self.track_t0 = None
            lim = np.array(getattr(self.bus, "pwm_ratio", [1.0] * 6)[:5]) * self.L["load_stop_ratio"]
            over = st.load[:5] > lim
            if over.any():
                self.load_t0 = self.load_t0 or now
                if now - self.load_t0 > 0.5:
                    j = int(np.argmax(st.load[:5] / lim))
                    self._set_fault(f"무리한 자세 (팔 뻗음 등)로 {JOINTS[j]} 부하 과다 — 모터 과부하 차단 전 정지 "
                                    f"({st.load[j] * 100:.0f}%, 한도 {lim[j] * 100:.0f}%)")
            else:
                self.load_t0 = None

            # 막힘 양보: 자리 유지 중 목표에 못 미친 관절은 목표를 현재 위치로 (계속 밀지 않기)
            if self.motion is None and self.jog is None:
                gap = np.abs(self.goal[:5] - st.q[:5]) > self.L.get("stall_yield_rad", 0.05)
                for j in range(5):
                    if gap[j]:
                        if np.isnan(self.stall_t0[j]):
                            self.stall_t0[j] = now
                        elif now - self.stall_t0[j] > self.L.get("stall_yield_time", 0.8):
                            self.stall_note = (f"{JOINTS[j]} 막힘 — 목표를 현재 위치로 바꿔 더 밀지 않음 "
                                               f"(목표 {self.goal[j]:.3f} / 실제 {st.q[j]:.3f} rad)")
                            log.warning(self.stall_note)
                            if now - self.stall_logged[j] > 600:        # 로그에는 관절마다 10분에 한 번
                                self.stall_logged[j] = now
                                self.on_event("angle", self.stall_note)
                            self.goal[j] = st.q[j]
                            self.stall_t0[j] = np.nan
                    else:
                        self.stall_t0[j] = np.nan
            else:
                self.stall_t0[:] = np.nan

            # 3) 허가·속도 배율
            stale = now - self.permit_t > self.L["permit_timeout"]
            target_scale = 0.0 if (stale or self.estop or self.fault) else self.permit_scale
            if self.slow_after_stop:
                target_scale = min(target_scale, 0.35)
            if target_scale < self.scale_now:
                self.scale_now = max(target_scale, self.scale_now - 6.0 * self.dt)
            else:
                self.scale_now = min(target_scale, self.scale_now + 1.5 * self.dt)
            if target_scale == 0.0:
                self.scale_now = 0.0 if (self.estop or self.fault or stale) else self.scale_now
                if self.motion is not None and self.scale_now == 0.0:
                    self.slow_after_stop = True        # 재개하면 다음 지점까지 저속

            # 4) 다음 목표 계산
            q_next = self.goal
            if self.motion is not None:
                m = self.motion
                m.t += self.dt * self.scale_now
                q_next = m.sample(m.t)
                if m.t >= m.total:
                    q_next = m.wp[-1]
                    m.finish("done")
                    self.motion = None
                    self.slow_after_stop = False
            elif self.jog is not None:
                q_next = self._jog_step(now)

            q_next = np.asarray(q_next, float).copy()
            q_next[5] = min(max(q_next[5], self.qmin[5]), self.qmax[5])
            try:
                self.check_q(q_next, q_from=self.goal)
            except LimitError as e:
                if self.motion is not None:
                    self.motion.finish("failed", str(e))
                    self.motion = None
                q_next = self.goal
            self.goal = np.asarray(q_next, float).copy()

        # 5) 목표 보내기 (자리 유지 중에도 계속 보낸다)
        try:
            self.bus.write_goal(self.goal)
            self.write_fail = 0
        except BusError as e:
            self.write_fail += 1
            self.comm_fail = max(self.comm_fail, self.write_fail)
            if self.write_fail >= self.L["comm_fail_max"]:
                self._set_fault(f"통신 오류: {e}")

    def _jog_step(self, now):
        vec, t_last = self.jog
        if now - t_last > 0.25:
            self.jog = None
            return self.goal
        if self.scale_now <= 0:
            return self.goal
        s = self.scale_now * self.dt
        if self.jog_mode == "joint":
            # 관절 조그: 관절 속도 = 속도 설정 x 최대 관절 속도, 그리퍼 0.8 rad/s. 한계에서 멈춘다.
            w = self.speed_ratio * self.L["joint_vel_max"]
            q = self.goal + np.r_[vec[:5] * w, vec[5] * 0.8] * s
            q = np.clip(q, np.minimum(self.qmin, self.goal), np.maximum(self.qmax, self.goal))
            hit = [(JOINTS + ["gripper"])[i] for i in range(6)
                   if vec[i] and (q[i] <= self.qmin[i] + 1e-6 or q[i] >= self.qmax[i] - 1e-6)]
            try:
                self.check_q(q, q_from=self.goal)
            except LimitError as e:
                self.jog_blocked = str(e)
                return self.goal
            self.jog_blocked = ("관절 한계: " + ", ".join(hit)) if hit else ""
            return q
        v = self.cart_speed * self.speed_ratio / self.L["speed_default_ratio"]
        v = min(v, self.L["cart_speed_max"])
        step = np.array([vec[0] * v, vec[1] * v, vec[2] * v, vec[3] * 0.5, 0.0, vec[4] * 0.8]) * s
        cand = self.jog_target + step
        try:
            q = self.solve(*cand[:4], q5=cand[4], grip=np.clip(cand[5], self.qmin[5], self.qmax[5]),
                           q_ref=self.goal)
        except (LimitError, K.IKError) as e:
            self.jog_blocked = str(e)
            return self.goal
        if np.abs(q[:5] - self.goal[:5]).max() > 0.08:         # 특이점 근처 급변 방지
            self.jog_blocked = "특이점 근처 (관절 급변)"
            return self.goal
        self.jog_blocked = ""
        cand[5] = q[5]
        self.jog_target = cand
        return q

    # ------------------------------------------------------------------ 상태
    def snapshot(self):
        st = self.state
        with self.lock:
            q = st.q.copy() if st is not None else np.zeros(6)
            return {
                "q": [round(float(v), 4) for v in q],
                "tcp": [round(float(v), 4) for v in K.fk(q)],
                "temp": [int(v) for v in st.temp] if st is not None else [],
                "load": [round(float(v), 2) for v in st.load] if st is not None else [],
                "volt": [round(float(v), 1) for v in st.volt] if st is not None else [],
                "torque": self.torque_on,
                "freedrive": self.freedrive,
                "estop": self.estop,
                "fault": self.fault,
                "permit": round(self.permit_scale, 2),
                "permit_reason": self.permit_reason,
                "scale": round(self.scale_now, 2),
                "speed_ratio": round(self.speed_ratio, 3),
                "speed_cap": self.L["speed_cap_ratio"],
                "motion": self.motion.name if self.motion is not None else ("jog" if self.jog else ""),
                "qmin": [round(float(v), 3) for v in self.qmin], "qmax": [round(float(v), 3) for v in self.qmax],
                "grip_force": self.gripper_force_pct(),
                "jog_blocked": self.jog_blocked,
                "stall_note": self.stall_note,
                "at_rest": self.at_rest(),
                "loop_ms": round(self.loop_ms, 1),
            }

    def link_points(self):
        """현재 FK 링크 점들 (안전 거리 계산용)."""
        st = self.state
        return K.fk_points(st.q) if st is not None else None
