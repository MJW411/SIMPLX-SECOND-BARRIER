"""다른 프로세스(노드)에 있는 객체를 기존 코드가 쓰던 모양 그대로 보여주는 대리 객체.

안전 관리자·작업 실행기·출입문 제어 같은 기존 코드는 ex.fault, io.healthy(), per.latest 처럼 객체를 직접 썼다.
노드를 나눈 뒤에도 그 코드를 고치지 않도록, 같은 이름의 속성·메서드를 토픽(상태)과 서비스(명령)로 잇는다.

상태가 끊기면 안전한 쪽으로 본다:
- 로봇 상태가 끊기면 fault 로 보인다 → 안전 관리자가 고장 정지
- 장치 상태가 끊기면 healthy()=False → 고장 정지
- 인식 결과가 끊기면 latest 가 갱신되지 않는다 → 안전 관리자의 신선도 검사(stale_timeout)로 고장 정지
"""
import math
import threading
import time
from types import SimpleNamespace

import numpy as np

from omx.robot import kinematics as K
from omx_interfaces.msg import HelmetState, IoState, People, Permit, RobotState, SafetyState

from .common import QOS_CMD, QOS_LATEST, RemoteError, now

BOOT_GRACE = 5.0          # 노드가 막 뜰 때 다른 노드 상태를 아직 못 받은 시간 (이 동안은 고장으로 보지 않음)
STATE_TIMEOUT = 1.0


class _Sub:
    def __init__(self, node, msg_type, topic):
        self.msg = None
        self.t = None
        self.t0 = now()
        node.create_subscription(msg_type, topic, self._cb, QOS_LATEST, callback_group=node.cb)

    def _cb(self, m):
        self.msg, self.t = m, now()

    def age(self):
        return math.inf if self.t is None else now() - self.t

    def fresh(self, timeout=STATE_TIMEOUT):
        return self.age() < timeout

    def booting(self):
        return self.t is None and now() - self.t0 < BOOT_GRACE


# ------------------------------------------------------------------------- 로봇
class RemoteMotion:
    """실행기 Motion 과 같은 모양 (status / reason / wait). 결과는 로봇 상태 토픽의 최근 동작 목록에서 읽는다."""

    def __init__(self, robot, mid, name):
        self.robot, self.id, self.name = robot, int(mid), name
        self._final = None

    def _lookup(self):
        m = self.robot.sub.msg
        if m is not None:
            for i, s, r in zip(m.motion_ids, m.motion_status, m.motion_reason):
                if i == self.id:
                    return s, r
        return None

    @property
    def status(self):
        if self._final:
            return self._final[0]
        r = self._lookup()
        if r is None:
            return "running" if self.robot.sub.fresh() else "cancelled"
        if r[0] != "running":
            self._final = r
        return r[0]

    @property
    def reason(self):
        r = self._final or self._lookup()
        return r[1] if r else ("로봇 노드 응답 없음" if not self.robot.sub.fresh() else "")

    def wait(self, timeout=None):
        end = None if timeout is None else time.monotonic() + timeout
        while self.status == "running":
            if end is not None and time.monotonic() >= end:
                break
            time.sleep(0.05)
        return self.status


class ExecutorProxy:
    """로봇 노드의 실행기 (안전 관리자·자동 보정·인식이 쓰는 부분)."""

    def __init__(self, node, robot_cfg, permit_pub=True):
        self.node = node
        self.cfg = robot_cfg                          # rest_pose 등
        self.sub = _Sub(node, RobotState, "/omx/robot_state")
        self.rpc = node.client("robot")
        if permit_pub:
            self.permit_pub = node.create_publisher(Permit, "/omx/permit", QOS_LATEST)
            from std_msgs.msg import Empty
            self.estop_pub = node.create_publisher(Empty, "/omx/estop", QOS_CMD)
            self._Empty = Empty

    # 상태 (끊기면 고장으로)
    @property
    def fault(self):
        m = self.sub.msg
        if not self.sub.fresh():
            return "" if self.sub.booting() else "로봇 노드 응답 없음"
        return m.fault

    @property
    def estop(self):
        m = self.sub.msg
        return bool(m.estop) if m is not None else False

    @property
    def torque_on(self):
        m = self.sub.msg
        return bool(m.torque_on) if m is not None and self.sub.fresh() else False

    @property
    def freedrive(self):
        m = self.sub.msg
        return bool(m.freedrive) if m is not None else False

    @property
    def motion(self):
        m = self.sub.msg
        return m.motion if m is not None else ""

    @property
    def state(self):
        m = self.sub.msg
        return SimpleNamespace(q=np.array(m.q, float)) if m is not None and m.ok else None

    def current_q(self):
        """실행기 current_q 와 같게: 그리퍼는 한계 안으로 (실제 위치가 끝까지 닫혀 한계를 살짝 넘을 때가 있다)."""
        st = self.state
        if st is None:
            raise RemoteError("로봇 상태 없음")
        q = st.q.copy()
        lo, hi = self.node.cfg.limits["gripper_limits"]
        q[5] = min(max(q[5], lo), hi)
        return q

    def link_points(self):
        st = self.state
        return K.fk_points(st.q) if st is not None and self.sub.fresh() else None

    # 명령
    after_permit = None                               # 안전 노드: 판단 한 번마다 상태 발행에 쓴다

    def permit(self, scale, reason=""):
        self.permit_pub.publish(Permit(t=now(), scale=float(scale), reason=str(reason)))
        if self.after_permit is not None:
            self.after_permit()

    def emergency_stop(self):
        self.estop_pub.publish(self._Empty())         # 빠른 길 (서비스 응답을 기다리지 않는다)
        self.rpc.send("estop")

    def stop_motion(self):
        self.rpc.send("stop_motion")

    def _require_ready(self):
        self.rpc.call("require_ready", timeout=3.0)

    def move_joints(self, q, name="move", speed_scale=1.0):
        r = self.rpc.call("move_joints", timeout=2.0, q=list(np.asarray(q, float)), name=name, speed_scale=speed_scale)
        return RemoteMotion(self, r["id"], name)

    def solve(self, x, y, z, pitch, q5=0.0, grip=None, q_ref=None):
        return np.array(self.rpc.call("solve", x=x, y=y, z=z, pitch=pitch, q5=q5, grip=grip), float)


# ------------------------------------------------------------------------- 인식
class PerceptionProxy:
    """안전 관리자가 쓰는 per.latest / per.cam.error. 인식 노드의 People 토픽에서 만든다."""

    def __init__(self, node):
        self.sub = _Sub(node, People, "/omx/people")
        self.cam = self
        self._cache = (None, None)

    @property
    def error(self):
        m = self.sub.msg
        return m.cam_error if m is not None else ""

    @property
    def latest(self):
        m = self.sub.msg
        if m is None:
            return None
        if self._cache[0] is m:
            return self._cache[1]
        persons = []
        for p in m.persons:
            persons.append(SimpleNamespace(
                box=tuple(float(v) for v in p.box), conf=float(p.conf), px=(float(p.px[0]), float(p.px[1])),
                pos=np.array(p.pos, float) if p.has_pos else None, dist=float(p.dist), cam_dist=float(p.cam_dist),
                npts=int(p.npts), in_zone=bool(p.in_zone), overlap=float(p.overlap), near=float(p.near),
                name=p.name, fallen=bool(p.fallen), poly=None))
        pc = SimpleNamespace(t=m.t, seq=m.seq, persons=persons, min_dist=float(m.min_dist),
                             calibrated=m.calibrated, infer_ms=m.infer_ms, shape=(int(m.height), int(m.width)),
                             brightness=float(m.brightness), model_ready=bool(m.model_ready))
        val = (SimpleNamespace(t=m.t, seq=m.seq), pc)
        self._cache = (m, val)
        return val


# ------------------------------------------------------------------------- 장치
class IoProxy:
    def __init__(self, node):
        self.sub = _Sub(node, IoState, "/omx/io_state")
        self.rpc = node.client("io")
        self.fire_ignore_before = 0.0                 # 이 시각 전에 보낸 화재 값은 믿지 않는다 (상황 종료 직후)

    def _m(self):
        return self.sub.msg if self.sub.fresh() else None

    def fire_live(self):
        """장치 노드에 지금 화재 센서 값을 직접 묻는다 (토픽이 아직 옛 값을 들고 있을 수 있는 순간용)."""
        return bool(self.rpc.call("status", timeout=2.0)["fire"])

    def healthy(self):
        if self.sub.booting():
            return True
        m = self._m()
        return bool(m.healthy) if m is not None else False

    def health_msg(self):
        m = self._m()
        if m is None:
            return "정상" if self.sub.booting() else "장치 노드 응답 없음"
        return m.health_msg

    def ready(self):
        m = self._m()
        return bool(m.ready) if m is not None else False

    def fire(self):
        m = self._m()
        if m is None or m.t < self.fire_ignore_before:
            return False
        return bool(m.fire)

    def sensor1(self):
        m = self._m()
        return bool(m.sensor1) if m is not None else False

    def sensor2(self):
        m = self._m()
        return bool(m.sensor2) if m is not None else False

    def door_open(self):
        m = self._m()
        return bool(m.door_open) if m is not None else False

    def set_door(self, open_):
        self.rpc.send("set_door", open=bool(open_))

    def set_conveyor(self, on):
        self.rpc.send("set_conveyor", on=bool(on))


# ------------------------------------------------------------------------- 안전 관리자 상태
class SafetyProxy:
    """작업 실행기·출입문·컨베이어·자동 보정이 읽는 safety.fire / maintenance / teach_mode / state / zone 등."""

    def __init__(self, node):
        self.sub = _Sub(node, SafetyState, "/omx/safety_state")
        self.rpc = node.client("safety")

    def _get(self, k, default):
        m = self.sub.msg
        return getattr(m, k) if m is not None else default

    @property
    def fire(self):
        return bool(self._get("fire", False))

    @property
    def maintenance(self):
        return bool(self._get("maintenance", False))

    @property
    def teach_mode(self):
        return bool(self._get("teach_mode", False))

    @property
    def state(self):
        return self._get("state", "STOP") if self.sub.fresh() else "STOP"

    @property
    def zone(self):
        return self._get("zone", "CLEAR")

    @property
    def human_alarm(self):
        return bool(self._get("human_alarm", False))

    @property
    def vanish(self):
        return SimpleNamespace(unresolved=[None] * int(self._get("vanish_unresolved", 0)),
                               pending=[None] * int(self._get("vanish_pending", 0)))

    @property
    def conveyor_alarm(self):
        return self._get("conveyor_alarm", "")

    @conveyor_alarm.setter
    def conveyor_alarm(self, msg):
        self.rpc.send("set_conveyor_alarm", msg=msg)


class RunnerProxy:
    """출입문 불빛·컨베이어가 보는 runner.running (로봇 노드의 작업 순서 실행 여부)."""

    def __init__(self, robot_sub):
        self.sub = robot_sub

    @property
    def running(self):
        m = self.sub.msg
        return bool(m.cycle_running) if m is not None and self.sub.fresh() else False

    @property
    def done(self):
        m = self.sub.msg
        return bool(m.cycle_done) if m is not None and self.sub.fresh() else False


class HelmetProxy:
    """출입문 제어가 보는 gate.wants_open(). 인식 노드의 헬멧 게이트 결과."""

    def __init__(self, node):
        self.sub = _Sub(node, HelmetState, "/omx/helmet")
        self.lock = threading.Lock()

    def wants_open(self, now_=None):
        m = self.sub.msg
        if m is None or not self.sub.fresh(2.0):      # 끊기면 닫힘 (안전 쪽)
            return False
        return now() < self.sub.t + float(m.open_left)

    def snapshot(self):
        return None
