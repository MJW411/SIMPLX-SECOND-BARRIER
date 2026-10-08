"""현장 입출력 단위 시험: 아두이노 통신, 컨베이어 제어(센서1 가동·센서2 정지·15초 이상), 출입문·불빛, 헬멧 판정.

설정은 사용자 설정(data/settings.json)과 무관하게 여기서 고정한다.
"""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.devices.io import ArduinoIO, VirtualIO  # noqa: E402
from omx.devices.site import SiteController, lamp_code  # noqa: E402
from omx.perception.helmet import HelmetGate  # noqa: E402
from omx.robot.task import ConveyorMonitor  # noqa: E402

fails = []


def check(c, what):
    print(("  OK  " if c else "  FAIL") + " " + what)
    if not c:
        fails.append(what)


CFG = {"conveyor": {"travel_timeout": 15.0}, "recording": {"pre_sec": 8},
       "door": {"close_sec": 5.0}, "helmet": {"conf": 0.5, "confirm_frames": 3}}


class Events:
    def __init__(self):
        self.got = []

    def emit(self, etype, msg, **kw):
        self.got.append((etype, msg, kw))
        return len(self.got)


# ---- 아두이노 통신 (가짜 직렬 포트)
class FakeSerial:
    def __init__(self):
        self.rx = b""
        self.tx = []
        self.closed = False

    def feed(self, b):
        self.rx += b

    def read(self, n):
        time.sleep(0.01)
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def write(self, b):
        self.tx.append(b)

    def close(self):
        self.closed = True


print("[아두이노 통신]")
fs = FakeSerial()
io = ArduinoIO(opener=lambda: fs)
check(io.healthy(), "첫 응답 전 부팅 유예 동안은 정상")
fs.feed(b"# omx_io v2\nS 1 0 0 0 1 4")
time.sleep(0.15)
check(not io.sensor1(), "줄이 끝나기 전에는 반영하지 않음")
fs.feed(b"2\nS 0 1 1 1 0 3")
time.sleep(0.15)
check(io.sensor1() and io.conveyor_on() and io.flame() == 42, "상태 줄 해석 (센서1·컨베이어·화염 값)")
fs.feed(b"21\ngarbage line\nS x y\n")
time.sleep(0.15)
check(io.sensor2() and io.fire() and io.door_open() and not io.conveyor_on() and io.flame() == 321,
      "잡음 줄은 무시하고 마지막 정상 줄 유지")
io.set_conveyor(True)
io.set_door(True)
io.set_led("O")
time.sleep(0.1)
check(fs.tx and fs.tx[-1] == b"C 1 1 O\n", f"명령 줄 즉시 전송 ({fs.tx[-1] if fs.tx else None})")
n = len(fs.tx)
time.sleep(0.5)
check(len(fs.tx) - n >= 2, "명령 줄을 하트비트로 계속 보냄 (200ms마다)")
io.last_rx = time.monotonic() - 2.0
check(not io.healthy() and "응답 없음" in io.health_msg(), "응답이 1초 끊기면 고장")
io.stop()
io2 = ArduinoIO(opener=lambda: fs)
io2.t_start -= 10
check(not io2.healthy(), "부팅 유예가 지나도 응답이 한 번도 없으면 고장")
io2.stop()


def failing_opener():
    raise IOError("없음")


io3 = ArduinoIO(opener=failing_opener)
time.sleep(0.1)
check("포트 열기 실패" in io3.err, "포트가 없으면 다시 시도하며 이유를 남김")
io3.stop()

# ---- 컨베이어 제어 (도착 센서 D12 하나 = sensor2)
print("[컨베이어]")
vio = VirtualIO()
ev = Events()
safety = SimpleNamespace(conveyor_alarm="", fire=False)
runner = SimpleNamespace(running=False)
cm = ConveyorMonitor(vio, ev, CFG, safety, runner, start=False)
t = 100.0
cm.step(t)
check(not vio.conveyor_on(), "작업 전에는 멈춤")
runner.running = True
cm.step(t + 0.1)
check(vio.conveyor_on(), "작업 시작 + 센서 비어 있음 → 가동")
vio.set_virtual(sensor2=True)
cm.step(t + 3.1)
check(not vio.s["conveyor"] and abs(cm.last_travel - 3.0) < 1e-6, "도착 센서 감지 → 정지, 도착 시간 기록")
cm.step(t + 10.0)
check(not vio.s["conveyor"] and not ev.got, "물품이 있는 동안은 계속 멈춤 (로봇이 집는 중)")
vio.set_virtual(sensor2=False)
cm.step(t + 10.1)
check(vio.conveyor_on(), "로봇이 집어 가 센서가 비면 다시 가동")
runner.running = False
cm.step(t + 11.0)
check(not vio.s["conveyor"], "작업이 끝나면 정지")
check(not ev.got, "정상 흐름에는 이벤트 없음")

runner.running = True
cm.step(t + 20.0)
cm.step(t + 34.9)
check(vio.conveyor_on() and not safety.conveyor_alarm, "15초 전에는 이상 아님")
cm.step(t + 35.1)
check(not vio.s["conveyor"] and safety.conveyor_alarm and ev.got and ev.got[-1][0] == "conveyor_jam"
      and ev.got[-1][2].get("clip"), "15초 동안 도착 센서에 물품이 없으면 정지 + 이상 이벤트(녹화)")
cm.step(t + 40.0)
check(not vio.s["conveyor"], "이상 해제 전에는 다시 돌리지 않음")
safety.conveyor_alarm = ""
cm.step(t + 41.0)
check(vio.conveyor_on(), "관리자 해제 뒤 (작업 중이면) 다시 가동")
safety.fire = True
cm.step(t + 42.0)
check(not vio.s["conveyor"], "화재 → 정지")
safety.fire = False
cm.step(t + 43.0)
CFG["conveyor"]["travel_timeout"] = 0
cm.step(t + 100.0)
check(vio.conveyor_on() and not safety.conveyor_alarm, "제한시간 0 = 이상 판정 끔")
CFG["conveyor"]["travel_timeout"] = 15.0
vio.set_virtual(conveyor=True, sensor2=True)
check(not vio.conveyor_on(), "가상 장치도 아두이노처럼 센서 감지 중엔 명령과 무관하게 멈춤")
vio.set_virtual(sensor2=False)

io5 = SimpleNamespace(rdy=False, cmds=[], ready=lambda: io5.rdy, sensor2=lambda: False, fire=lambda: False,
                      set_conveyor=lambda on: io5.cmds.append(on))
cm5 = ConveyorMonitor(io5, Events(), CFG, SimpleNamespace(conveyor_alarm="", fire=False),
                      SimpleNamespace(running=True), start=False)
cm5.step(0.0)
check(io5.cmds == [], "아두이노 첫 응답 전에는 아무 명령도 안 함")
io5.rdy = True
cm5.step(0.1)
check(io5.cmds == [True], "첫 응답 뒤 작업 중이면 가동")

# ---- 불빛
print("[불빛]")
check(lamp_code(True, "STOP", True, True) == "Y", "화재 노랑이 가장 우선")
check(lamp_code(False, "SLOW", False, True) == "O", "사람 감속 거리 → 주황 점멸")
check(lamp_code(False, "STOP", False, False) == "O", "사람 정지 거리 → 주황 점멸")
check(lamp_code(False, "CLEAR", True, True) == "O", "사라진 뒤 다시 안 나타남 → 주황 점멸")
check(lamp_code(False, "CLEAR", False, True) == "B", "작업 중 → 하늘색")
check(lamp_code(False, "CLEAR", False, False) == "G", "작업 끝·대기 → 녹색")

# ---- 출입문
print("[출입문]")
vio = VirtualIO()
gate = SimpleNamespace(open_until=0.0, wants_open=lambda now: now < gate.open_until, snapshot=lambda: {})
saf = SimpleNamespace(fire=False, zone="CLEAR", vanish=SimpleNamespace(unresolved=[]))
runner = SimpleNamespace(running=True)
ev = Events()
site = SiteController(vio, saf, runner, CFG, gate, ev)
now = time.monotonic()
site.step(now)
check(not vio.door_open() and vio.led() == "B", "평소 닫힘, 작업 중 하늘색")
gate.open_until = now + 5
site.step(now + 1)
check(vio.door_open(), "헬멧 확인 → 열림")
site.step(now + 5.1)
check(not vio.door_open(), "확인 5초 뒤 닫힘")
saf.fire = True
site.step(now + 6)
check(vio.door_open() and vio.led() == "Y", "화재 → 열림 + 노랑")
saf.fire = False
vio.set_virtual(fire=False)
site.step(now + 7)
check(not vio.door_open(), "화재 상황 종료 → 닫힘")
site.open_door("admin")
site.step(time.monotonic() + 1)
check(vio.door_open() and ev.got and ev.got[-1][0] == "door_manual", "수동 열기 → 열림 + 사용자 판단 기록")

# ---- 헬멧 판정
print("[헬멧]")
D = lambda cls, conf: SimpleNamespace(cls=cls, name={0: "helmet", 1: "head"}[cls], conf=conf, box=(0, 0, 10, 10))  # noqa: E731
cam = SimpleNamespace(latest=lambda: None, error="", fps_meas=0.0)
det = SimpleNamespace(names={0: "helmet", 1: "head"}, model_name="m", infer_ms=0.0)
g = HelmetGate(cam, det, CFG)
t = 1000.0
g.evaluate([D(0, 0.9)], t)
g.evaluate([D(0, 0.9)], t + 0.2)
check(not g.wants_open(t + 0.2), "2프레임으로는 열지 않음")
g.evaluate([D(0, 0.9)], t + 0.4)
check(g.wants_open(t + 0.4) and not g.wants_open(t + 5.5), "연속 3프레임 헬멧 → 5초 열림")
g2 = HelmetGate(cam, det, CFG)
for i in range(5):
    g2.evaluate([D(0, 0.9), D(1, 0.8)], t + i * 0.2)
check(not g2.wants_open(t + 1) and "미착용" in g2.state, "헬멧과 맨머리가 같이 보이면 열지 않음")
g3 = HelmetGate(cam, det, CFG)
for i in range(5):
    g3.evaluate([D(0, 0.3)], t + i * 0.2)
check(not g3.wants_open(t + 1), "확신도 50% 미만 헬멧은 무시")
g4 = HelmetGate(SimpleNamespace(latest=lambda: None, error="", fps_meas=0.0),
                SimpleNamespace(names={0: "head", 1: "helmet"}, model_name="v3", infer_ms=0.0), CFG)
check(g4.helmet_cls == 1 and g4.head_cls == 0, "클래스 번호가 다른 모델도 이름으로 찾음")

print()
print("실패 %d건" % len(fails) if fails else "모두 통과")
sys.exit(1 if fails else 0)
