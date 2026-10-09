"""2026-10-08 수정 사항 시험: 모델 준비·프레임 시각 신선도·낙상·재가동 구역 검사·화재 홈 재시도,
그리퍼 목표 유지·쓰기 실패 고장·작업 정지 경합."""
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.config import Config  # noqa: E402
from omx.perception.people import Percept, Person  # noqa: E402
from omx.robot.dxl_bus import BusError, SimBus  # noqa: E402
from omx.robot.executor import Executor  # noqa: E402
from omx.robot.task import CycleRunner  # noqa: E402
from omx.safety.manager import SafetyManager  # noqa: E402

C = Config()
fails = []


def check(c, what):
    print(("  OK   " if c else "  FAIL ") + what)
    if not c:
        fails.append(what)


# ------------------------------------------------------------------ 안전 관리자
emitted = []
fake_ex = SimpleNamespace(fault="", estop=False, torque_on=True, cfg=C["robot"], permit=lambda *a: None,
                          stop_motion=lambda *a: None, emergency_stop=lambda: None)
fake_io = SimpleNamespace(fire=lambda: False, healthy=lambda: True, health_msg=lambda: "", set_door=lambda *a: None)
fake_per = SimpleNamespace(latest=None, cam=SimpleNamespace(error=""))
sm = SafetyManager(C, fake_ex, fake_per, fake_io, SimpleNamespace(emit=lambda t, *a, **k: emitted.append(t)))
seq = [0]
STOP = sm.profile()["stop"]


def person(dist, fallen=False, in_zone=True, x=320):
    return Person(box=(x - 20, 200, x + 20, 300), conf=0.9, px=(x, 250), pos=np.array([0.3, 0.0, 0.0]),
                  dist=dist, cam_dist=dist, in_zone=in_zone, name="fall-down" if fallen else "human", fallen=fallen)


def feed(persons=(), n=1, ready=True, t=None):
    for _ in range(n):
        seq[0] += 1
        ps = list(persons)
        zd = [p.dist for p in ps if p.in_zone]
        fake_per.latest = (None, Percept(time.monotonic() if t is None else t, seq[0], persons=ps,
                                         min_dist=min(zd) if zd else float("inf"), model_ready=ready))
        sm._step()


print("[인식 모델 준비]")
feed(n=3)
sm.restart()
check(not sm.stop_latched, "정상 시작")
feed(n=2, ready=False)
check(sm.stop_latched.startswith("인식 모델 없음") and sm.state == "STOP", f"모델 없음 → 고장 정지 ({sm.stop_latched})")
ok, probs = sm.restart()
check(not ok and any("모델" in p for p in probs), f"모델 없으면 재가동 불가 {probs}")
feed(n=2)
ok, _ = sm.restart()
check(ok, "모델이 준비되면 재가동 가능")

print("[프레임 시각 신선도]")
feed(n=1, t=time.monotonic() - 2.0)          # 2초 전에 찍은 프레임이 이제 도착
sm._step()
check(sm.stop_latched.startswith("인식 멈춤"), f"늦게 도착한 오래된 프레임 → 인식 멈춤 정지 ({sm.stop_latched})")
feed(n=2)
sm.restart()

print("[낙상: 알림만, 정지는 거리로]")
emitted.clear()
far = STOP * 20
feed([person(far, fallen=True)], n=C["safety"]["fall_confirm_frames"] + 2)
check("person_fall" in emitted, "넘어진 사람이 계속 보이면 낙상 이벤트")
check(not sm.stop_latched and sm.state in ("NORMAL", "SLOW"), f"구역 밖·먼 낙상만으로는 정지하지 않음 ({sm.state})")
check(emitted.count("person_fall") == 1, "낙상 이벤트는 한 번만")
feed([person(STOP * 0.5, fallen=True)], n=4)
check(sm.state == "STOP" and sm.stop_latched.startswith("사람 근접"), f"넘어진 사람이 정지 거리 안 → 정지 ({sm.stop_latched})")
feed(n=40)
check(not sm.fallen, "넘어진 사람이 안 보이면 낙상 상태 해제")

print("[재가동: 근접 판정 해제 전에는 안 됨]")
check(len(sm.vanish.pending) + len(sm.vanish.unresolved) > 0, "출구가 아닌 곳에서 사라진 사람 → 사라짐 판정 (설계대로)")
sm.clear_vanish()
sm.zone = "STOP"
sm.min_dist = 99.0
ok, probs = sm.restart()
check(not ok and any("근접 판정" in p for p in probs), f"거리는 멀지만 판정이 아직 STOP → 거부 {probs}")
feed(n=4)
ok, probs = sm.restart()
check(ok, f"판정이 해제되면 재가동 {probs}")

print("[화재 홈 복귀: 취소되면 다시 시도]")
calls = []


class M:
    def __init__(self):
        self.status = "running"


sent_q = []


def mv(q, name):
    calls.append(name)
    sent_q.append(np.array(q, float))
    return M()


fake_ex.move_joints = mv
fake_ex.current_q = lambda: np.array([0.1, -1.5, 1.5, 1.5, 0.0, 0.3])
sm.home_q_fn = lambda: [0.0, -0.5, 0.3, 1.0, 0.0, -0.02]      # 저장된 홈의 그리퍼 값이 한계 밖 (실기에서 거부된 경우)
sm._fire_home()
check(calls == ["fire_home"], "처음 한 번 보냄")
check(abs(sent_q[0][5] - 0.3) < 1e-9 and abs(sent_q[0][1] + 0.5) < 1e-9, "화재 홈은 팔만 — 그리퍼는 지금 위치 유지")
sm._fire_home()
check(len(calls) == 1, "진행 중이면 다시 보내지 않음")
sm.fire_homing.status = "cancelled"
sm._fire_retry_t -= 3
sm._fire_home()
check(len(calls) == 2, "취소되면 2초 뒤 다시 보냄")

print("[사라짐: 화면 밖으로 나감 / 들려 나감 / 진짜 사라짐]")
from omx.safety.vanish import VanishTracker  # noqa: E402


def P(box):
    x1, y1, x2, y2 = box
    return SimpleNamespace(pos=None, px=((x1 + x2) / 2, (y1 + y2) / 2), dist=0.5, in_zone=True, box=box)


def run_frames(vt, frames, t0=0.0, dt=0.1):
    new = []
    for k, persons in enumerate(frames):
        new += vt.update(SimpleNamespace(persons=persons, shape=(480, 640)), t0 + k * dt)
    return new


grace = C["safety"]["vanish_grace"]
tail = [[]] * int(grace / 0.1 + 5)                 # 유예시간이 지나도록 빈 화면
# 1) 손이 위에서 들어왔다가 위로 나감 (마지막 박스가 위 가장자리에 닿음, 중심은 가장자리에서 멂)
vt = VanishTracker(C)
hand = [[P((300, 0, 360, 200))]] * 10
new = run_frames(vt, hand + tail)
check(not new and not vt.unresolved, "위 가장자리로 나간 손 → 사라짐 아님")
# 2) 피규어(F)를 손(H)이 집어 위로 나감: F 가 화면 가운데서 끊기고, 겹친 H 가 이어서 위로 나감
vt = VanishTracker(C)
F = (300, 250, 330, 330)
seq = [[P(F)]] * 8                                  # 피규어 확립
seq += [[P(F), P((290, 120, 350, 300))]] * 8        # 손이 내려와 피규어를 덮음
seq += [[P((290, 100, 350, 280))], [P((292, 60, 352, 240))], [P((294, 20, 354, 200))], [P((296, 0, 356, 170))]] + [[P((296, 0, 356, 150))]] * 6   # 피규어 끊김, 손이 위로
seq += tail
new = run_frames(vt, seq)
check(not new and not vt.unresolved, f"손이 피규어를 들고 나감 → 사라짐 아님 (미해결 {len(vt.unresolved)})")
# 3) 피규어가 가운데서 그냥 사라짐 (겹친 검출 없음)
vt = VanishTracker(C)
new = run_frames(vt, [[P(F)]] * 8 + tail)
check(len(new) == 1 and len(vt.unresolved) == 1, "가운데서 그냥 사라짐 → 사라짐 (가려짐 의심)")
# 4) 손이 덮었지만 손은 화면 안에 남음 → 사라짐 유지
vt = VanishTracker(C)
seq = [[P(F)]] * 8 + [[P(F), P((290, 120, 350, 300))]] * 8 + [[P((290, 120, 350, 300))]] * int(grace / 0.1 + 5)
new = run_frames(vt, seq)
check(len(vt.unresolved) == 1, "손이 덮고 화면 안에 남음 → 사라짐 유지")

# ------------------------------------------------------------------ 실행기
print("[그리퍼 목표 유지]")
bus = SimBus()
ex = Executor(bus, C["robot"], C.limits, on_event=lambda *a: None)
ex.start()
ex.set_torque(True)
ex.goal[5] = 0.2                       # 물체를 쥔 상태: 목표가 실제보다 더 닫힌 쪽
bus.state.q[5] = 0.45
ex.emergency_stop()
ex.reset()
check(abs(ex.goal[5] - 0.2) < 1e-9, f"고장 해제 후 그리퍼 목표 유지 ({ex.goal[5]:.2f})")
ex.set_torque(True)
check(abs(ex.goal[5] - 0.2) < 1e-9, "토크 켜기 후 그리퍼 목표 유지")

print("[쓰기 실패 반복 → 통신 고장]")
orig = bus.write_goal


def bad_write(q):
    raise BusError("쓰기 실패 시험")


bus.write_goal = bad_write
t_end = time.monotonic() + 2.0
while not ex.fault and time.monotonic() < t_end:
    time.sleep(0.02)
check(ex.fault.startswith("통신 오류"), f"읽기는 되고 쓰기만 계속 실패 → 고장 ({ex.fault})")
bus.write_goal = orig
ex.reset()

print("[작업 정지 직후 다음 동작이 나가지 않음]")
ex.set_torque(True)
ex.permit(1.0)
sent = []


class FakeEx:
    def __init__(self):
        self.state = SimpleNamespace(q=np.zeros(6))

    def current_q(self):
        return np.zeros(6)

    def stop_motion(self):
        pass

    def move_joints(self, q, name, speed_scale=1.0):
        sent.append(name)
        m = SimpleNamespace(name=name, status="done", reason="", wait=lambda t=None: "done")
        return m


gate = threading.Event()
release = threading.Event()
teach = SimpleNamespace(snapshot=lambda: {"sequence": [{"type": "move", "pose": "a"}, {"type": "move", "pose": "b"}],
                                          "gripper": {}},
                        pose_q=lambda n: [0.0] * 6, home_q=lambda: None)
cr = CycleRunner(FakeEx(), teach, None, SimpleNamespace(emit=lambda *a, **k: None), C.limits)
orig_wait = cr._wait


def slow_wait(m):
    orig_wait(m)
    gate.set()                         # 첫 단계 끝: 여기서 정지 명령이 끼어든다
    release.wait(2.0)


cr._wait = slow_wait
cr.start()
gate.wait(2.0)
cr.stop("시험")
release.set()
cr.thread.join(2.0)
check(sent == ["move a"], f"정지 뒤 다음 단계 동작을 보내지 않음 ({sent})")
check(cr.status.startswith("정지"), f"상태: {cr.status}")

print("[반복 횟수 → 작업 완료]")
ev2 = []
cr2 = CycleRunner(FakeEx(), teach, None, SimpleNamespace(emit=lambda t, m, **k: ev2.append((t, m))), C.limits)
cr2.start(max_cycles=2)
cr2.thread.join(5.0)
check(cr2.cycles == 2 and cr2.done and ("cycle_done", "작업 완료 — 2회 반복") in ev2, f"2회 마치면 완료 + 이벤트 ({cr2.status}, {ev2})")
check(cr2.snapshot()["done"] and cr2.snapshot()["max_cycles"] == 2, "상태에 완료·목표 횟수")
cr2.teach = SimpleNamespace(snapshot=lambda: {"sequence": [{"type": "pause", "value": 0.5}], "gripper": {}})
cr2.start(max_cycles=1)
check(not cr2.done, "다음 작업을 시작하면 완료 표시 해제")
cr2.thread.join(5.0)
cr2.stop()
check(not cr2.done, "정지를 누르면 완료 표시 해제")
check(C.limits.get("cycle_max_minutes", 0) == 0, "시간 한도 없음 (0)")

print("[일시 정지 → 재개: 멈춘 단계부터, 반복 횟수 이어서]")


class PauseEx(FakeEx):
    """동작은 release 될 때까지 진행 중, stop_motion 이면 취소."""

    def __init__(self):
        super().__init__()
        self.cur = None

    def stop_motion(self):
        if self.cur is not None and self.cur.status == "running":
            self.cur.status = "cancelled"

    def move_joints(self, q, name, speed_scale=1.0):
        sent.append(name)
        m = SimpleNamespace(name=name, status="running", reason="")

        def wait(t=None, m=m):
            time.sleep(0.02)
            if m.status == "running" and auto_done.is_set():
                m.status = "done"
            return m.status
        m.wait = wait
        self.cur = m
        return m


def until(cond, t=3.0):
    end = time.monotonic() + t
    while not cond() and time.monotonic() < end:
        time.sleep(0.02)
    return cond()


sent.clear()
auto_done = threading.Event()
s2 = threading.Event()
io_p = SimpleNamespace(sensor2=s2.is_set, set_conveyor=lambda on: None)
teach_p = SimpleNamespace(snapshot=lambda: {"sequence": [{"type": "move", "pose": "a"}, {"type": "wait_sensor2"},
                                                         {"type": "move", "pose": "b"}], "gripper": {}},
                          pose_q=lambda n: [0.0] * 6, home_q=lambda: None)
crp = CycleRunner(PauseEx(), teach_p, io_p, SimpleNamespace(emit=lambda *a, **k: None), C.limits)
crp.start(max_cycles=2)
until(lambda: sent == ["move a"])
check(crp.pause("컨베이어 이상") and crp.paused, "동작 중 일시 정지")
time.sleep(0.3)
check(crp.running and sent == ["move a"] and crp.snapshot()["paused"], f"일시 정지 중: 작업은 살아 있고 새 동작 없음 ({sent})")
check(crp.snapshot()["pause_why"] == "컨베이어 이상", "상태에 일시 정지 이유")
auto_done.set()
crp.resume("컨베이어 이상 해제")
check(until(lambda: crp.step_idx == 1), "재개 → 취소됐던 1단계를 다시 하고 2단계(도착 센서 대기)로")
check(sent == ["move a", "move a"], f"멈춘 단계(1단계)부터 다시 ({sent})")
crp.pause("컨베이어 이상")                          # 도착 센서 대기 중 일시 정지
time.sleep(0.2)
s2.set()                                            # 일시 정지 중엔 센서가 와도 다음 단계로 가지 않음
time.sleep(0.2)
check(sent == ["move a", "move a"] and crp.step_idx == 1, "일시 정지 중엔 다음 단계로 안 넘어감")
crp.resume()
crp.thread.join(5.0)
check(crp.cycles == 2 and crp.done, f"재개 뒤 반복 횟수 이어서 완료 ({crp.cycles}회, {crp.status})")
check(sent == ["move a", "move a", "move b", "move a", "move b"], f"동작 순서 ({sent})")
check(not crp.paused and not crp.pause("x"), "작업이 끝나면 일시 정지 안 됨")
s2.clear()
auto_done.clear()
sent.clear()
crp.start(max_cycles=1)
until(lambda: sent == ["move a"])
crp.pause("컨베이어 이상")
crp.stop("정지 버튼")
crp.thread.join(2.0)
check(not crp.running and crp.status.startswith("정지") and not crp.resume(), "일시 정지 중 정지 버튼 → 작업 끝 (재개 안 됨)")

ex.shutdown()
print()
print("실패 %d건" % len(fails) if fails else "모두 통과")
sys.exit(1 if fails else 0)
