"""안전 노드 — 판단은 여기 한 곳에서만 한다 (기존 SafetyManager 그대로).

입력: /omx/people (인식), /omx/robot_state (로봇), /omx/io_state (장치)
출력: /omx/permit 20Hz (로봇 실행기 허가), /omx/safety_state 20Hz, /omx/estop (비상정지), 이벤트
이 프로세스가 죽으면 허가가 끊겨 로봇이 permit_timeout(0.3초) 안에 선다. 장치 노드는 safety_state 가
끊긴 것을 보고 컨베이어를 멈춘다.
"""
import json

import rclpy
from std_msgs.msg import String

from omx.safety.manager import SafetyManager
from omx_interfaces.msg import SafetyState

from .common import QOS_LATEST, QOS_SNAPSHOT, ChangePublisher, OmxNode, now, spin
from .proxies import ExecutorProxy, IoProxy, PerceptionProxy


class SafetyNode(OmxNode):
    def __init__(self):
        super().__init__("safety")
        C = self.cfg
        self.ex = ExecutorProxy(self, C["robot"])
        self.per = PerceptionProxy(self)
        self.io = IoProxy(self)
        self.robot = self.client("robot")
        sm = self.sm = SafetyManager(C, self.ex, self.per, self.io, self.events)
        # 홈 자세는 로봇 노드 스냅샷(티칭 저장소)에서 미리 받아 둔다 — 화재 때 판단 루프가 서비스 응답을 기다리지 않게
        self.teach = {}
        self.create_subscription(String, "/omx/robot/snapshot", self._robot_snap, QOS_SNAPSHOT, callback_group=self.cb)
        sm.home_q_fn = self._home_q
        sm.on_fire_start.append(lambda: self.robot.send("runner_stop", why="화재"))
        sm.guard_person_m = float(C["safety"].get("guard_person_m", 0) or 0)
        # 상태는 판단(20Hz)마다 확인해 바뀌었을 때만, 아니면 0.2초마다 보낸다 (장치 노드는 1초 끊기면 안전 상태)
        self.state_out = ChangePublisher(self.create_publisher(SafetyState, "/omx/safety_state", QOS_LATEST), 0.2)
        self.ex.after_permit = self._publish_state
        self.on_config.append(self._config)

        def user(a):
            return a.get("user", "admin")
        self.handlers.update({
            "estop": lambda a: sm.emergency_stop(a.get("source", "비상정지 버튼")),
            "restart": lambda a: dict(zip(("ok", "problems"), sm.restart(user(a)))),
            "restart_blockers": lambda a: sm.restart_blockers(),
            "clear_vanish": lambda a: sm.clear_vanish(user(a)),
            "set_maintenance": lambda a: sm.set_maintenance(bool(a.get("on"))),
            "set_teach_mode": lambda a: sm.set_teach_mode(bool(a.get("on")), user(a)),
            "end_fire": self._end_fire,
            "conveyor_clear": lambda a: setattr(sm, "conveyor_alarm", ""),
            "set_conveyor_alarm": lambda a: setattr(sm, "conveyor_alarm", a.get("msg", "")),
            "set_guard": lambda a: setattr(sm, "guard_person_m", float(a.get("meters", 0))),
            # 웹의 동작 명령 전 검사: 토픽 캐시가 아니라 지금 값 (정비 모드를 켠 직후의 명령도 막는다)
            "guard": lambda a: {"fire": sm.fire, "maintenance": sm.maintenance},
        })
        self.start_snapshot(sm.snapshot)
        sm.start()
        self.log.info("안전 노드 시작 (프로필 %s)", C["safety"]["profile"])

    def _end_fire(self, a):
        """화재 상황 종료: 센서 값을 장치 노드에 직접 확인하고, 종료 전에 보낸 옛 화재 값으로 다시 화재가 걸리지 않게 한다."""
        if self.io.fire_live():
            raise RuntimeError("화재 센서가 아직 감지 중 — 센서가 꺼진 뒤 상황 종료")
        with self.sm.lock:
            self.io.fire_ignore_before = now()
            return self.sm.end_fire(a.get("user", "admin"))

    def _robot_snap(self, msg):
        try:
            self.teach = json.loads(msg.data).get("teach") or {}
        except ValueError:
            pass

    def _home_q(self):
        t = self.teach
        h = t.get("home")
        return t.get("poses", {}).get(h, {}).get("q") if h else None

    def _config(self, section):
        if section == "safety":
            self.sm.guard_person_m = float(self.cfg["safety"].get("guard_person_m", 0) or 0)

    def _publish_state(self):
        sm = self.sm
        with sm.lock:
            v = (sm.state, sm.zone, sm.fire, sm.maintenance, sm.teach_mode, sm.stop_latched,
                 float(sm.min_dist), len(sm.vanish.unresolved), len(sm.vanish.pending), sm.conveyor_alarm, sm.fallen)
        key = v[:6] + (round(v[6], 2) if v[6] != float("inf") else -1,) + v[7:]
        self.state_out.offer(key, lambda: SafetyState(
            t=now(), state=v[0], zone=v[1], fire=v[2], maintenance=v[3], teach_mode=v[4], stop_latched=v[5],
            min_dist=v[6], vanish_unresolved=v[7], vanish_pending=v[8], conveyor_alarm=v[9], fallen=v[10]))


def main():
    rclpy.init()
    spin(SafetyNode())


if __name__ == "__main__":
    main()
