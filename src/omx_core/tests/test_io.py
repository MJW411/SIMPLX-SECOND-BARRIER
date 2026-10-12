"""현장 입출력 단위 시험: 아두이노 통신, 컨베이어 제어(센서1 가동·센서2 정지·15초 이상), 출입문·불빛, 헬멧 판정.

설정은 사용자 설정(data/settings.json)과 무관하게 여기서 고정한다.
"""
import copy
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import yaml

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


CFG = {"conveyor": {"travel_timeout": 15.0, "resume_pose": "place"}, "recording": {"pre_sec": 8},
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
io.set_arrive_stop(False)
time.sleep(0.1)
check(fs.tx[-1] == b"C 1 1 O 0\n", f"도착 센서 무시 → 다섯째 값 0 ({fs.tx[-1]})")
io.set_arrive_stop(True)
time.sleep(0.1)
check(fs.tx[-1] == b"C 1 1 O\n", "기본으로 돌리면 예전 형식 그대로")
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

# ---- 컨베이어 제어 (투입 센서 D11 = sensor1 이면 출발, 도착 센서 D12 = sensor2 면 정지)
print("[컨베이어]")
vio = VirtualIO()
ev = Events()
safety = SimpleNamespace(conveyor_alarm="", fire=False, state="NORMAL")
runner = SimpleNamespace(running=False)
cm = ConveyorMonitor(vio, ev, CFG, safety, runner, start=False)
t = 100.0
cm.step(t)
check(not vio.conveyor_on(), "작업 전에는 멈춤")
runner.running = True
cm.step(t + 0.1)
check(not vio.conveyor_on(), "작업 중이라도 투입 센서에 물품이 없으면 멈춤 (빈 컨베이어 안 돌림)")
vio.set_virtual(sensor1=True)
cm.step(t + 0.1)
check(vio.conveyor_on(), "투입 센서에 물품 → 출발")
vio.set_virtual(sensor1=False)
cm.step(t + 1.0)
check(vio.conveyor_on(), "물품이 투입 센서를 지나가도 도착할 때까지 계속")
vio.set_virtual(sensor2=True)
cm.step(t + 3.1)
check(not vio.s["conveyor"] and abs(cm.last_travel - 3.0) < 1e-6, "도착 센서 감지 → 정지, 도착 시간 기록")
cm.step(t + 10.0)
check(not vio.s["conveyor"] and not ev.got, "물품이 있는 동안은 계속 멈춤 (로봇이 집는 중)")
vio.set_virtual(sensor2=False)
cm.step(t + 10.1)
check(not vio.conveyor_on(), "로봇이 집어 가도 새 물품이 올라오기 전에는 멈춤")
vio.set_virtual(sensor1=True)
cm.step(t + 10.2)
check(vio.conveyor_on(), "새 물품이 투입 센서에 → 다시 가동")
vio.set_virtual(sensor1=False)
runner.running = False
cm.step(t + 11.0)
check(not vio.s["conveyor"], "작업이 끝나면 정지")
check(not ev.got, "정상 흐름에는 이벤트 없음")

runner.running = True
vio.set_virtual(sensor1=True)
cm.step(t + 20.0)
vio.set_virtual(sensor1=False)
cm.step(t + 34.9)
check(vio.conveyor_on() and not safety.conveyor_alarm, "15초 전에는 이상 아님")
cm.step(t + 35.1)
check(not vio.s["conveyor"] and safety.conveyor_alarm and ev.got and ev.got[-1][0] == "conveyor_jam"
      and ev.got[-1][2].get("clip"), "15초 동안 도착 센서에 물품이 없으면 정지 + 이상 이벤트(녹화)")
cm.step(t + 40.0)
check(not vio.s["conveyor"], "이상 해제 전에는 다시 돌리지 않음")
safety.conveyor_alarm = ""
vio.set_virtual(sensor1=True)
cm.step(t + 41.0)
check(vio.conveyor_on(), "관리자 해제 뒤 (작업 중 + 물품) 다시 가동")
safety.state = "STOP"
cm.step(t + 41.5)
check(not vio.s["conveyor"], "사람 정지(안전 STOP) → 컨베이어도 정지")
safety.state = "NORMAL"
vio.set_virtual(sensor1=False)
cm.step(t + 41.6)
check(vio.conveyor_on() and cm.t_run == t + 41.6, "재가동되면 (투입 센서가 비어도) 이어서 가동, 제한시간 새로 셈")
vio.set_virtual(sensor1=True)
safety.fire = True
cm.step(t + 42.0)
check(not vio.s["conveyor"], "화재 → 정지")
safety.fire = False
cm.step(t + 43.0)
CFG["conveyor"]["travel_timeout"] = 0
cm.step(t + 100.0)
check(vio.conveyor_on() and not safety.conveyor_alarm, "제한시간 0 = 이상 판정 끔")
CFG["conveyor"]["travel_timeout"] = 15.0
vio.set_virtual(sensor1=False)
runner.running = False
cm.step(t + 101.0)                                  # 작업 끝 → 정지 (새 작업으로 시험)
runner.running = True
CFG["conveyor"]["start_sensor"] = False
cm.step(t + 102.0)
check(vio.conveyor_on(), "start_sensor=false: 예전 방식 (작업 중 도착 센서가 비면 가동)")
CFG["conveyor"]["start_sensor"] = True
vio.set_virtual(conveyor=True, sensor2=True)
check(not vio.conveyor_on(), "가상 장치도 아두이노처럼 센서 감지 중엔 명령과 무관하게 멈춤")
vio.set_virtual(sensor2=False)

# ---- 컨베이어 이상 자동 해제 + 로봇 작업 일시 정지·재개
vj = VirtualIO()
evj = Events()
sj = SimpleNamespace(conveyor_alarm="", fire=False)
rj = SimpleNamespace(running=True, stopped=[], log=[])
rj.stop = lambda why: rj.stopped.append(why)
rj.pause = lambda why, pose="": (rj.log.append(("pause", why)), setattr(rj, "pose", pose))
rj.resume = lambda why: rj.log.append(("resume", why))
cj = ConveyorMonitor(vj, evj, CFG, sj, rj, start=False)
vj.set_virtual(sensor1=True)
cj.step(200.0)
vj.set_virtual(sensor1=False)
cj.step(216.0)
check(sj.conveyor_alarm and not vj.conveyor_on(), "15초 넘게 도착 안 함 → 이상")
check(cj.history and cj.history[-1]["jam"] and cj.snapshot()["history"][-1]["s"] == 15.0, "차트 기록: 이상은 제한시간 막대(빨강)")
check(rj.log == [("pause", "컨베이어 이상")] and rj.stopped == [], "이상이면 로봇 작업은 끝내지 않고 일시 정지")
check(rj.pose == "place" and yaml.safe_load(open(Path(__file__).parent.parent / "config/default.yaml"))["conveyor"]["resume_pose"] == "place",
      "재개할 때 place 자세에서 물품 대기 (기본 설정 resume_pose: place)")
cj.step(217.0)
check(sj.conveyor_alarm and not vj.conveyor_on(), "이상인 동안 다시 돌리지 않음")
vj.set_virtual(sensor1=True)
cj.step(218.0)
check(not sj.conveyor_alarm and "투입 센서" in evj.got[-1][1], "투입 센서에 새 물품 → 자동 해제")
check(rj.log[-1][0] == "resume", "해제되면 로봇 작업 재개")
check(vj.conveyor_on(), "해제 즉시 컨베이어 다시 가동")
cj.step(234.0)                                      # 그 물품도 15초 넘게 도착 안 함 (투입 센서엔 계속 있음)
check(sj.conveyor_alarm and rj.log[-1] == ("pause", "컨베이어 이상"), "다시 막힘 → 이상·일시 정지")
cj.step(235.0)
check(sj.conveyor_alarm, "투입 센서에 물품이 계속 있기만 하면 저절로 풀지 않음 (새로 올라와야)")
vj.set_virtual(sensor2=True)
cj.step(236.0)
check(not sj.conveyor_alarm and evj.got[-1][0] == "conveyor_clear" and rj.log[-1][0] == "resume",
      "물품이 도착 센서에 닿아도 자동 해제·재개")
vj.set_virtual(sensor2=False)
cj.step(236.5)
cj.step(252.0)
check(sj.conveyor_alarm, "또 막힘 → 이상")
cj.step(252.5)                                      # 안전 노드 상태에 이상이 실린 뒤
n = len(rj.log)
sj.conveyor_alarm = ""                              # 관리자 해제 버튼 (안전 노드가 이상 문구를 지움)
cj.step(253.0)
check(rj.log[n:] == [("resume", "컨베이어 이상 해제")], "관리자 해제 버튼으로 풀려도 재개")
cj.step(254.0)
check(rj.log[n + 1:] == [], "재개는 한 번만")
vj.set_virtual(sensor1=False)
cj.step(254.5)
cj.step(270.0)
check(sj.conveyor_alarm, "다시 막힘 → 이상")
rj.running = False
cj.step(271.0)
check(sj.conveyor_alarm, "작업이 끝나도 이상은 유지")
rj.running = True
cj.step(272.0)
check(not sj.conveyor_alarm and "새로 시작" in evj.got[-1][1], "작업을 새로 시작하면 자동 해제")

# ---- 시간 자동 해제: 아무 조건이 없어도 auto_clear_sec 뒤 저절로 풀고 재개 (수동 해제 버튼도 그대로)
check(yaml.safe_load(open(Path(__file__).parent.parent / "config/default.yaml"))["conveyor"]["auto_clear_sec"] == 0,
      "기본 설정: 시간 자동 해제 끔 (팝업 재가동 버튼으로)")
cft = copy.deepcopy(CFG); cft["conveyor"]["auto_clear_sec"] = 10.0
vt = VirtualIO()
evt = Events()
st_ = SimpleNamespace(conveyor_alarm="", fire=False)
rt = SimpleNamespace(running=True, log=[])
rt.pause = lambda why, pose="": rt.log.append(("pause", why))
rt.resume = lambda why: rt.log.append(("resume", why))
ct = ConveyorMonitor(vt, evt, cft, st_, rt, start=False)
vt.set_virtual(sensor1=True)
ct.step(500.0)
vt.set_virtual(sensor1=False)                       # 물품을 손으로 치움 → 투입·도착 센서 모두 변화 없음
ct.step(516.0)
check(st_.conveyor_alarm and rt.log == [("pause", "컨베이어 이상")], "막힘 → 이상·일시 정지")
ct.step(525.0)
check(st_.conveyor_alarm, "10초 전에는 유지")
ct.step(526.0)
check(not st_.conveyor_alarm and "자동 해제" in evt.got[-1][1] and rt.log[-1][0] == "resume", "10초 지나면 저절로 해제·재개")
ct.step(527.0)
check(not st_.conveyor_alarm and rt.log.count(("resume", "컨베이어 이상 해제")) == 1, "한 번만 재개")
cfo = copy.deepcopy(CFG); cfo["conveyor"]["auto_clear_sec"] = 0
so = SimpleNamespace(conveyor_alarm="", fire=False)
vo = VirtualIO()
co = ConveyorMonitor(vo, Events(), cfo, so, SimpleNamespace(running=True), start=False)
vo.set_virtual(sensor1=True); co.step(600.0); vo.set_virtual(sensor1=False); co.step(616.0)
co.step(700.0)
check(so.conveyor_alarm, "auto_clear_sec: 0 → 시간으로는 안 풀림 (예전처럼)")

# ---- 도착 센서 고장 시 임시: 작업 중이면 센서와 상관없이 계속 돎
va = VirtualIO()
cfa = copy.deepcopy(CFG)
cfa["conveyor"].update(start_sensor=False, travel_timeout=0, arrive_sensor=False)
ra = SimpleNamespace(running=True)
ca = ConveyorMonitor(va, Events(), cfa, SimpleNamespace(conveyor_alarm="", fire=False), ra, start=False)
va.set_virtual(sensor1=False, sensor2=True)                # 도착 센서가 계속 '있음'으로 고장
ca.step(300.0)
check(va.conveyor_on() and not va.arrive_stop, "arrive_sensor=false: 작업 시작하면 바로 가동, 도착 센서 감지돼도 계속")
ca.step(400.0)
check(va.conveyor_on(), "제한시간 0 → 오래 돌아도 이상 아님")
ra.running = False
ca.step(401.0)
check(not va.conveyor_on(), "작업이 끝나면 정지")
cfa["conveyor"]["arrive_sensor"] = True
ra.running = True
ca.step(402.0)
check(not va.conveyor_on() and va.arrive_stop, "arrive_sensor=true 로 되돌리면 도착 센서 감지 시 정지")

io5 = SimpleNamespace(rdy=False, cmds=[], ready=lambda: io5.rdy, sensor1=lambda: True, sensor2=lambda: False, fire=lambda: False,
                      set_conveyor=lambda on: io5.cmds.append(on))
cm5 = ConveyorMonitor(io5, Events(), CFG, SimpleNamespace(conveyor_alarm="", fire=False),
                      SimpleNamespace(running=True), start=False)
cm5.step(0.0)
check(io5.cmds == [], "아두이노 첫 응답 전에는 아무 명령도 안 함")
io5.rdy = True
cm5.step(0.1)
check(io5.cmds == [True], "첫 응답 뒤 작업 중 + 물품이면 가동")

# ---- 불빛
print("[불빛]")
check(lamp_code(True, "STOP", True, True) == "Y", "화재 노랑이 가장 우선")
check(lamp_code(False, "SLOW", False, True) == "O", "사람 감속 거리 → 주황")
check(lamp_code(False, "STOP", False, False) == "R", "사람 정지 거리 → 빨강")
check(lamp_code(False, "CLEAR", False, False, human_alarm=True) == "R", "사람이 떠나도 재가동 전까지 빨강")
check(lamp_code(False, "CLEAR", True, True) == "R", "사라진 뒤 확인 전 → 빨강")
check(lamp_code(False, "CLEAR", False, True) == "B", "작업 중 → 하늘색")
check(lamp_code(False, "CLEAR", False, False) == "G", "대기 → 백색(G)")
check(lamp_code(False, "CLEAR", False, False, done=True) == "C", "정한 횟수 완료 → 초록(C)")
check(lamp_code(False, "SLOW", False, False, done=True) == "O", "완료 중이라도 사람 감속이 먼저")

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


def stub_det(names, model_name="m"):
    inv = {v: k for k, v in names.items()}
    return SimpleNamespace(names=names, model_name=model_name, infer_ms=0.0, lane_ms=[0.0, 0.0], ready=True,
                           class_ids=lambda wanted: [inv[n] for n in wanted if n in inv])


D = lambda cls, conf: SimpleNamespace(cls=cls, name={0: "helmet", 1: "head"}[cls], conf=conf, box=(0, 0, 10, 10))  # noqa: E731
cam = SimpleNamespace(latest=lambda: None, error="", fps_meas=0.0)
det = stub_det({0: "helmet", 1: "head"})
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
g4 = HelmetGate(SimpleNamespace(latest=lambda: None, error="", fps_meas=0.0), stub_det({0: "head", 1: "helmet"}, "v3"), CFG)
check(g4.helmet_ids == {1} and g4.head_ids == {0}, "클래스 번호가 다른 모델도 이름으로 찾음")

# ---- v2 통합 모델 (fall-down / helmet / human): 사람 머리 영역의 헬멧
V2 = {0: "fall-down", 1: "helmet", 2: "human"}
V = lambda cls, box, conf=0.9: SimpleNamespace(cls=cls, name=V2[cls], conf=conf, box=box)  # noqa: E731
HUMAN, HEAD_HELMET, HAND_HELMET = (100, 100, 200, 400), (130, 85, 170, 125), (120, 330, 160, 370)


def run_gate(dets, match="head", n=4):
    cfg = copy.deepcopy(CFG)
    cfg["helmet"]["match"] = match
    gg = HelmetGate(cam, stub_det(V2, "v2"), cfg)
    for i in range(n):
        gg.evaluate(dets, t + i * 0.2)
    return gg


check(run_gate([V(2, HUMAN), V(1, HEAD_HELMET)]).wants_open(t + 0.7), "v2 head: 사람 머리에 헬멧 → 열림")
gx = run_gate([V(2, HUMAN), V(1, HAND_HELMET)])
check(not gx.wants_open(t + 0.7) and "미착용" in gx.state, "v2 head: 손에 든 헬멧(아래쪽) → 닫힘")
check(not run_gate([V(1, HEAD_HELMET)]).wants_open(t + 0.7), "v2 head: 사람 없이 헬멧만 → 닫힘")
check(not run_gate([V(2, HUMAN), V(2, (300, 100, 400, 400)), V(1, HEAD_HELMET)]).wants_open(t + 0.7),
      "v2 head: 두 사람 중 한 명만 착용 → 닫힘")
check(run_gate([V(0, (100, 300, 400, 360)), V(1, (360, 315, 390, 345))]).wants_open(t + 0.7),
      "v2 head: 넘어진 사람은 박스 전체에서 헬멧을 찾음")
check(run_gate([V(2, HUMAN), V(1, HAND_HELMET)], match="presence").wants_open(t + 0.7),
      "v2 presence: 사람과 헬멧이 함께 보이면 열림")
check(not run_gate([V(1, HEAD_HELMET)], match="presence").wants_open(t + 0.7), "v2 presence: 헬멧만 → 닫힘")
gp = run_gate([V(2, HUMAN)], match="presence")
check(not gp.wants_open(t + 0.7) and gp.state == "헬멧 미착용", f"v2 presence: 헬멧 없는 사람 → '헬멧 미착용' 표시 ({gp.state})")
gp = run_gate([V(2, HUMAN), V(1, HAND_HELMET)], match="presence")
check(gp.state == "헬멧 착용 확인 - 문 열림", f"v2 presence: 착용 확인 문구 ({gp.state})")
OTHER = (300, 100, 400, 400)
gp = run_gate([V(2, HUMAN), V(2, OTHER), V(1, HEAD_HELMET)], match="presence")
check(not gp.wants_open(t + 0.7) and gp.state == "헬멧 미착용", f"v2 presence: 착용자 + 미착용자 → 닫힘 ({gp.state})")
check(run_gate([V(2, HUMAN), V(2, OTHER), V(1, HEAD_HELMET), V(1, (330, 85, 370, 125))], match="presence").wants_open(t + 0.7),
      "v2 presence: 두 사람 모두 착용 → 열림")
check(not run_gate([V(2, HUMAN), V(2, (150, 100, 250, 400)), V(1, HEAD_HELMET)], match="presence").wants_open(t + 0.7),
      "v2 presence: 겹쳐 선 두 사람에 헬멧 하나 → 닫힘 (헬멧 하나는 한 사람에게만)")
check(not run_gate([V(2, HUMAN), V(1, (450, 380, 490, 420))], match="presence").wants_open(t + 0.7),
      "v2 presence: 사람과 떨어진 곳(바닥·선반)의 헬멧 → 닫힘")
gp = run_gate([V(2, HUMAN, 0.3)], match="presence")
check(gp.state == "사람 확인 중", f"v2 presence: 사람인지 불확실(확신도 낮음)하면 '사람 확인 중' ({gp.state})")

print()
print("실패 %d건" % len(fails) if fails else "모두 통과")
sys.exit(1 if fails else 0)
