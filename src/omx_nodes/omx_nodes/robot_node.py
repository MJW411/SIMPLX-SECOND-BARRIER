"""로봇 노드 — 로봇 명령은 이 한 통로로만 (기존 Executor 50Hz + 티칭 저장소 + 작업 순서 실행기).

입력: /omx/permit (안전 노드 허가: 계속 와야 움직인다), /omx/estop, /omx/jog (웹 조그)
출력: /omx/robot_state 50Hz (자세·토크·고장·최근 동작 결과), 스냅샷 5Hz, 이벤트
관리 명령: 웹 제어판의 로봇·티칭·작업 API (처리 내용은 기존 web/app.py 와 같다)
이 프로세스가 죽으면 모터는 마지막 작은 목표에서 자리를 유지한다 (토크 유지).
"""
import argparse
import collections
import itertools
import sys
import threading
import time

import numpy as np
import rclpy
from std_msgs.msg import Empty

from omx.robot.dxl_bus import BusError, DxlBus, SimBus
from omx.robot.executor import Executor, LimitError
from omx.robot.task import CycleRunner, pickplace_template
from omx.robot.teach import TeachStore, grip_target
from omx_interfaces.msg import Permit, RobotState

from .common import QOS_CMD, QOS_LATEST, ChangePublisher, OmxNode, device_lock, loop_thread, now, spin
from .proxies import IoProxy, SafetyProxy

GRIP_MIN_SPAN = 0.15        # 열림·닫힘 끝이 이보다 가까우면 열기·닫기가 사실상 움직이지 않는다
JOINTS6 = ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")


class RobotNode(OmxNode):
    def __init__(self, sim=False):
        super().__init__("robot")
        C = self.cfg
        r = C["robot"]
        ids = [r["ids"][k] for k in JOINTS6]
        backend = "sim" if sim or r["backend"] == "sim" else "real"
        if backend == "real":
            device_lock("robot")
            try:
                bus = DxlBus(r["port"], r["baud"], ids, C.limits["gripper_current_ma"])
            except BusError as e:
                self.log.error("로봇 연결 실패 (%s) — 가상 로봇으로 시작", e)
                bus, backend = SimBus(), "sim (로봇 연결 실패)"
        else:
            bus = SimBus()
        self.backend = backend
        self.ex = ex = Executor(bus, r, C.limits)
        ex.on_event = self._robot_event
        self.teach = TeachStore(C.path("poses.json"))
        self.io = IoProxy(self)
        self.safety = SafetyProxy(self)
        self.runner = CycleRunner(ex, self.teach, self.io, self.events, C.limits, self.safety)
        # 원격 동작 핸들용: 명령으로 시작한 동작의 최근 결과
        self.motions = collections.OrderedDict()
        self.mid = itertools.count(1)
        self.mlock = threading.Lock()

        self.create_subscription(Permit, "/omx/permit", self._permit, QOS_LATEST, callback_group=self.cb)
        self.create_subscription(Empty, "/omx/estop", lambda _: ex.emergency_stop(), QOS_CMD, callback_group=self.cb)
        # 상태: 움직이는 동안은 제어 주기(50Hz)마다, 가만히 있으면 0.2초마다 (인식의 링크 거리·안전 판단용)
        self.state_out = ChangePublisher(self.create_publisher(RobotState, "/omx/robot_state", QOS_LATEST), 0.2)
        loop_thread(self._publish_state, r["control_hz"], "robot-state")
        self._register()
        ex.start()
        self.start_snapshot(self.snapshot)
        self.log.info("로봇 노드 시작 (%s)", backend)

    # ------------------------------------------------------------------ 입력
    def _permit(self, m):
        ex = self.ex
        ex.permit(m.scale, m.reason)
        with ex.lock:                       # 허가 시각은 안전 노드가 보낸 시각 (전달이 늦으면 그만큼 오래된 허가)
            ex.permit_t = min(ex.permit_t, m.t)

    def _robot_event(self, kind, msg):
        if kind == "estop":
            return                          # 비상정지는 안전 노드가 기록한다 (중복 방지)
        self.events.emit("robot" if kind == "robot" else f"robot_{kind}", msg,
                         level="warn" if kind in ("fault", "temp", "angle") else "info")

    # ------------------------------------------------------------------ 출력
    def _track(self, m):
        with self.mlock:
            i = next(self.mid)
            self.motions[i] = m
            while len(self.motions) > 16:
                self.motions.popitem(last=False)
        return {"id": i}

    def _publish_state(self):
        ex = self.ex
        st = ex.state
        with self.mlock:
            ms = list(self.motions.items())
        q = [float(v) for v in (st.q if st is not None else np.zeros(6))]
        motion = ex.motion.name if ex.motion is not None else ("jog" if ex.jog else "")
        stat = [m.status for _, m in ms]
        key = (tuple(round(v, 4) for v in q), st is not None, ex.torque_on, ex.freedrive, ex.estop, ex.fault, motion,
               self.runner.running, self.runner.done, tuple(stat))
        self.state_out.offer(key, lambda: RobotState(
            t=now(), q=q, ok=st is not None, torque_on=ex.torque_on, freedrive=ex.freedrive, estop=ex.estop,
            fault=ex.fault, motion=motion, cycle_running=self.runner.running, cycle_done=self.runner.done,
            motion_ids=[i for i, _ in ms], motion_status=stat, motion_reason=[m.reason or "" for _, m in ms]))

    def snapshot(self):
        return {"robot": self.ex.snapshot(), "teach": self.teach.snapshot(), "cycle": self.runner.snapshot(),
                "robot_backend": self.backend + (f" {self.ex.bus.port_name.split('/')[-1][:24]}"
                                                 if hasattr(self.ex.bus, "port_name") else ""),
                "busy": bool(self.ex.motion) or self.runner.running}

    # ------------------------------------------------------------------ 명령 (기존 web/app.py 의 로봇·티칭·작업 처리)
    def _register(self):
        ex, teach, runner, C = self.ex, self.teach, self.runner, self.cfg

        def home_target():
            q = teach.home_q()
            if q is None:
                q = list(ex.cfg["rest_pose"]) + [ex.current_q()[5]]
                q[0] = ex.current_q()[0]
            q = np.array(q, float)
            q[5] = ex.current_q()[5]
            return q

        def park(a):
            def job():
                q = np.array(list(ex.cfg["rest_pose"]) + [ex.current_q()[5]])
                q[0] = ex.current_q()[0]          # 베이스는 돌리지 않는다 (접힌 채로 제자리)
                m = ex.move_joints(q, "park")
                if m.wait(120) == "done":
                    time.sleep(0.8)
                    before = ex.state.q.copy()
                    ex.set_torque(False)
                    time.sleep(2.0)
                    if np.abs(ex.state.q[:5] - before[:5]).max() > 0.1:
                        ex.set_torque(True)
                        self.events.emit("robot", "토크를 끄자 팔이 처짐 — 토크 다시 켬", level="warn")
                    else:
                        self.events.emit("robot", "휴식 자세로 이동 후 토크 끔", level="info")
            ex._require_ready()
            threading.Thread(target=job, daemon=True).start()

        def goto(a):
            q = np.array(teach.pose_q(a["pose"]), float)
            q[5] = ex.current_q()[5]                 # 자세는 팔만 — 그리퍼는 그리퍼 명령으로만
            return self._track(ex.move_joints(q, f"goto {a['pose']}"))

        def joints(a):
            q = [float(v) for v in a["q"]]
            cur = ex.current_q()
            tgt = np.array(q + [cur[5]] if len(q) == 5 else q, float)
            return self._track(ex.move_joints(tgt, "joints"))

        def gripper(a):
            step = {"preset": a["preset"]} if a.get("preset") else \
                {"value": a["percent"] if "percent" in a else a.get("value", "open")}
            if a.get("force") is not None:
                step["force"] = a["force"]
            target, force, _ = teach.grip_action(step)
            ex.set_gripper_force(force)
            return self._track(ex.move_gripper(target))

        def grips_save(a):
            g = teach.snapshot()["gripper"]
            pos = grip_target(g, a["percent"]) if a.get("percent") is not None else float(ex.state.q[5])
            return teach.save_grip(a["name"], pos, a.get("force", g.get("force", 50)))

        def grips_default_force(a):
            with teach.lock:
                teach.data["gripper"]["force"] = int(min(100, max(5, float(a["force"]))))
                teach._save()

        def save_grip_range(open_=None, close=None):
            """그리퍼 가동 범위(열림 끝·닫힘 끝, rad) 저장. 그리퍼 기구 한계 안으로 맞추고, 둘 사이 0.15 rad 이상."""
            lo, hi = C.limits["gripper_limits"]
            with teach.lock:
                g = teach.data["gripper"]
                valid_before = abs(g["open"] - g["close"]) >= GRIP_MIN_SPAN
                o = g["open"] if open_ is None else round(min(hi, max(lo, float(open_))), 3)
                c = g["close"] if close is None else round(min(hi, max(lo, float(close))), 3)
                one_side = open_ is None or close is None
                if abs(o - c) < GRIP_MIN_SPAN and not (one_side and not valid_before):
                    raise ValueError(f"열림 끝({o:.3f})과 닫힘 끝({c:.3f})이 너무 가깝습니다 — {GRIP_MIN_SPAN} rad 이상 떨어져야 "
                                     "열기·닫기가 움직입니다. 그리퍼를 끝까지 연(닫은) 뒤 다시 저장하세요")
                g["open"], g["close"] = o, c
                teach._save()
            self.log.info("그리퍼 가동 범위: 닫힘 %.3f ~ 열림 %.3f rad", c, o)

        def pose_gripper(a):
            which = a["which"]
            if which not in ("open", "close"):
                raise ValueError("which 는 open 또는 close")
            v = float(ex.state.q[5])
            save_grip_range(**({"open_": v} if which == "open" else {"close": v}))

        def cycle_start(a):
            at = int(a.get("start_at", 0))
            sec = "before" if a.get("section") == "before" else "main"
            runner.start(int(a.get("max_cycles", 0)), at, sec)
            snap = teach.snapshot()
            n, nb = len(snap["sequence"]), len(snap.get("before_start", []))
            self.events.emit("cycle_start", f"{a.get('user', 'admin')}: 작업 시작 ({n}단계"
                             + (f", 시작 전 {nb}단계" if nb and (sec == "before" or not at) else "")
                             + (f", {a['max_cycles']}회" if a.get("max_cycles") else "")
                             + (f", 시작 전 {at + 1}단계부터" if sec == "before" else f", {at + 1}단계부터" if at else "") + ")")

        def cycle_stop(a):
            was = runner.running
            runner.stop()
            if was:
                self.events.emit("cycle_stop", f"{a.get('user', 'admin')}: 작업 정지 ({runner.cycles}회 완료)")
            return was

        def stop(a):
            runner.stop()
            ex.stop_motion()

        def move_joints(a):
            return self._track(ex.move_joints(np.array(a["q"], float), a.get("name", "move"),
                                              float(a.get("speed_scale", 1.0))))

        def solve(a):
            return ex.solve(a["x"], a["y"], a["z"], a["pitch"], q5=a.get("q5", 0.0), grip=a.get("grip"))

        def shutdown(a):
            ex.shutdown()                   # 토크는 켠 채 자리 유지
            threading.Thread(target=lambda: (time.sleep(0.5), rclpy.shutdown()), daemon=True).start()

        self.handlers.update({
            "torque": lambda a: ex.set_torque(bool(a.get("on")), force=bool(a.get("force"))),
            "park": park,
            "home": lambda a: self._track(ex.move_joints(home_target(), "home")),
            "goto": goto,
            "joints": joints,
            "freedrive": lambda a: (runner.stop("직접 교시") if a.get("on") else None,
                                    ex.set_freedrive(bool(a.get("on"))))[1],
            "align": lambda a: self._track(ex.align(a.get("target", "down"))),
            "tcp": lambda a: self._track(ex.move_linear(a["x"], a["y"], a["z"], a["pitch"], name="tcp")),
            "gripper": gripper,
            "gripper_force": lambda a: ex.set_gripper_force(a["force"]),
            "grips_save": grips_save,
            "grips_delete": lambda a: teach.delete_grip(a["name"]),
            "grips_default_force": grips_default_force,
            "estop": lambda a: ex.emergency_stop(),
            "reset": lambda a: ex.reset(),
            "stop": stop,
            "stop_motion": lambda a: ex.stop_motion(),
            "speed": lambda a: ex.set_speed(float(a["ratio"])),
            "poses_save": lambda a: teach.save_pose(a["name"], ex.state.q),
            "poses_delete": lambda a: teach.delete_pose(a["name"]),
            "poses_home": lambda a: teach.set_home(a["name"]),
            "pose_gripper": pose_gripper,
            "gripper_range": lambda a: save_grip_range(a.get("open"), a.get("close")),
            "sequence": lambda a: teach.set_sequence(a["steps"], a.get("before_start")),
            "sequence_template": lambda a: teach.set_sequence(pickplace_template(teach.snapshot()["poses"])),
            "cycle_start": cycle_start,
            "cycle_step": lambda a: runner.run_one(int(a["index"]), "before" if a.get("section") == "before" else "main"),
            "cycle_stop": cycle_stop,
            "runner_stop": lambda a: runner.stop(a.get("why", "정지 명령")),
            "runner_pause": lambda a: runner.pause(a.get("why", "일시 정지"), a.get("pose", "")),
            "runner_resume": lambda a: runner.resume(a.get("why", "")),
            "grip_check": lambda a: setattr(runner, "grip_check", bool(a.get("on"))),
            "jog": lambda a: ex.jog_cmd(a["jog"], a.get("mode", "cart")),
            "move_joints": move_joints,
            "solve": solve,
            "require_ready": lambda a: ex._require_ready(),
            "status": lambda a: {"torque_on": ex.torque_on, "freedrive": ex.freedrive, "estop": ex.estop,
                                 "fault": ex.fault, "busy": bool(ex.motion) or runner.running},
            "home_q": lambda a: teach.home_q(),
            "shutdown": shutdown,
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="로봇 없이 가상 로봇으로")
    a, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    spin(RobotNode(sim=a.sim))


if __name__ == "__main__":
    main()
