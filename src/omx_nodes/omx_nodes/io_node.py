"""장치 노드 — 아두이노(또는 가상 장치) 시리얼 포트를 이 프로세스만 잡는다.

- 출입문·상태 불빛 (기존 SiteController): 화재·헬멧 확인·수동 열기
- 컨베이어 감시 (기존 ConveyorMonitor): 작업 중 + 도착 센서 비면 가동, 제한시간 초과 시 이상
- 안전 노드 상태(/omx/safety_state)가 1초 넘게 끊기면 컨베이어를 멈추고 문은 화재일 때만 연다.
  (아두이노도 젯슨 명령이 1초 끊기면 스스로 컨베이어 정지·문 닫힘)
출력: /omx/io_state 20Hz, 스냅샷 5Hz, 이벤트
"""
import argparse
import sys
import threading
import time

import rclpy

from omx.devices.io import make_io
from omx.devices.site import SiteController
from omx.robot.task import ConveyorMonitor
from omx_interfaces.msg import IoState, RobotState

from .common import QOS_LATEST, ChangePublisher, OmxNode, device_lock, now, spin
from .proxies import HelmetProxy, RunnerProxy, SafetyProxy, _Sub

SAFETY_TIMEOUT = 1.0


class IoNode(OmxNode):
    def __init__(self, virtual=False):
        super().__init__("io")
        C = self.cfg
        if C["devices"]["backend"] == "arduino" and not virtual:
            device_lock("arduino")
        self.io = io = make_io(C, force_virtual=virtual)
        self.safety = SafetyProxy(self)
        self.runner = RunnerProxy(_Sub(self, RobotState, "/omx/robot_state"))
        self.gate = HelmetProxy(self)
        self.site = SiteController(io, self.safety, self.runner, C, self.gate, self.events)
        self.conveyor = ConveyorMonitor(io, self.events, C, self.safety, self.runner, start=False)
        self.watchdog = False
        self.state_out = ChangePublisher(self.create_publisher(IoState, "/omx/io_state", QOS_LATEST), 0.2)
        threading.Thread(target=self._loop, daemon=True, name="site").start()

        def virtual_set(a):
            if not hasattr(io, "set_virtual"):
                raise PermissionError("가상 장치 모드가 아님")
            io.set_virtual(**a)

        def then_publish(fn):                  # 명령으로 바뀐 값은 다음 주기를 기다리지 않고 바로 알린다
            def run(a):
                r = fn(a)
                self._publish_state()
                return r
            return run

        self.handlers.update({
            "set_virtual": then_publish(virtual_set),
            "door_open": lambda a: self.site.open_door(a.get("user", "admin")),
            "set_door": then_publish(lambda a: io.set_door(bool(a.get("open")))),
            "set_conveyor": then_publish(lambda a: io.set_conveyor(bool(a.get("on")))),
            "status": lambda a: io.snapshot(),
        })
        self.start_snapshot(self.snapshot)
        self.log.info("장치 노드 시작 (%s)", type(io).__name__)

    def _loop(self):
        """출입문·불빛 10Hz, 컨베이어 20Hz. 안전 노드가 끊기면 안전 상태."""
        n = 0
        while rclpy.ok():
            t = time.monotonic()
            try:
                lost = not self.safety.sub.fresh(SAFETY_TIMEOUT) and not self.safety.sub.booting()
                if lost:
                    if not self.watchdog:
                        self.log.warning("안전 노드 상태 끊김 — 컨베이어 정지, 문은 화재일 때만 열림")
                    self.io.set_conveyor(False)
                    self.conveyor.on, self.conveyor.t_run = False, None
                    self.io.set_door(self.io.fire())
                    self.io.set_led("O")
                else:
                    self.conveyor.step(t)
                    if n % 2 == 0:
                        self.site.step(t)
                self.watchdog = lost
                self._publish_state()
            except Exception:
                self.log.exception("site/conveyor")
            n += 1
            time.sleep(max(0.0, 0.05 - (time.monotonic() - t)))

    def _publish_state(self):
        io = self.io
        v = (io.ready(), io.healthy(), io.health_msg(), io.sensor1(), io.sensor2(), io.fire(), io.door_open(),
             io.conveyor_on(), io.led())
        self.state_out.offer(v, lambda: IoState(t=now(), ready=v[0], healthy=v[1], health_msg=v[2], sensor1=v[3],
                                                sensor2=v[4], fire=v[5], door_open=v[6], conveyor_on=v[7], led=v[8]))

    def snapshot(self):
        return {"devices": self.io.snapshot(), "conveyor": self.conveyor.snapshot(), "site": self.site.snapshot(),
                "safety_link_lost": self.watchdog}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--virtual", action="store_true", help="아두이노 없이 가상 장치로")
    a, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    spin(IoNode(virtual=a.virtual))


if __name__ == "__main__":
    main()
