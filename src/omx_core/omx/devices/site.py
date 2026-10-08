"""현장 출력 정리: 출입문과 상태 불빛을 한 곳에서 정해 장치(아두이노·가상)에 계속 보낸다.

출입문: 화재(상황 종료 전까지) 또는 헬멧 확인 후 close_sec 동안, 또는 관리자가 제어판에서 연 뒤 close_sec 동안 열림.
불빛 (우선순위 순):
  Y 노랑        — 화재 (아두이노는 불꽃을 보면 젯슨을 기다리지 않고 스스로 노랑)
  O 주황 점멸   — 사람이 로봇에 접근해 감지되는 동안(감속·정지 거리 안), 또는 사라진 뒤 다시 나타나지 않은 동안
  B 하늘색      — 작업 중 (작업 순서 실행 중)
  G 녹색        — 작업 끝·대기
"""
import logging
import threading
import time

log = logging.getLogger("site")


def lamp_code(fire, zone, vanish_unresolved, running):
    if fire:
        return "Y"
    if zone in ("SLOW", "STOP") or vanish_unresolved:
        return "O"
    if running:
        return "B"
    return "G"


class SiteController:
    PERIOD = 0.1

    def __init__(self, io, safety, runner, cfg, gate=None, events=None):
        self.io, self.safety, self.runner, self.cfg, self.gate, self.events = io, safety, runner, cfg, gate, events
        self.manual_until = 0.0
        self.door = False
        self.led = "G"
        self._run = True

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name="site").start()
        return self

    def open_door(self, user):
        """관리자가 제어판에서 문을 연다 (헬멧 확인 없이). close_sec 뒤 닫힌다."""
        self.manual_until = time.monotonic() + self.cfg["door"]["close_sec"]
        if self.events is not None:
            self.events.emit("door_manual", f"{user}: 출입문 수동 열림 ({self.cfg['door']['close_sec']:.0f}초)", level="info")

    def step(self, now):
        fire = self.safety.fire or self.io.fire()
        door = fire or now < self.manual_until or (self.gate is not None and self.gate.wants_open(now))
        led = lamp_code(fire, self.safety.zone, bool(self.safety.vanish.unresolved), self.runner.running)
        if door != self.door:
            log.info("출입문 %s", "열림" if door else "닫힘")
        self.door, self.led = door, led
        self.io.set_door(door)
        self.io.set_led(led)

    def _loop(self):
        while self._run:
            try:
                self.step(time.monotonic())
            except Exception:
                log.exception("site")
            time.sleep(self.PERIOD)

    def snapshot(self):
        return {"door_wanted": self.door, "led": self.led,
                "manual_left": round(max(0.0, self.manual_until - time.monotonic()), 1),
                "close_sec": self.cfg["door"]["close_sec"],
                "helmet": self.gate.snapshot() if self.gate is not None else None}
