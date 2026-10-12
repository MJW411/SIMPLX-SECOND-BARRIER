"""핵심 로직 단위 시험: 기구학, 사라짐 판정, 실행기 한계·허가."""
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.config import Config  # noqa: E402
from omx.robot import kinematics as K  # noqa: E402
from omx.robot.dxl_bus import SimBus  # noqa: E402
from omx.robot.executor import Executor, LimitError  # noqa: E402
from omx.safety.vanish import VanishTracker  # noqa: E402

C = Config()
fails = []


def check(c, what):
    print(("  OK  " if c else "  FAIL") + " " + what)
    if not c:
        fails.append(what)


# ---- 기구학
rng = np.random.default_rng(0)
worst = 0
for _ in range(500):
    q = np.array([rng.uniform(-2, 2), rng.uniform(-1.4, 1.3), rng.uniform(-1.3, 1.5), rng.uniform(-1.5, 1.8), 0])
    x, y, z, p = K.fk(q)
    try:
        q2 = K.ik(x, y, z, p)
    except K.IKError:            # 최대 리치 경계의 부동소수 오차
        continue
    worst = max(worst, np.abs(K.fk(q2)[:3] - [x, y, z]).max())
check(worst < 0.002, f"FK/IK 왕복 오차 {worst * 1000:.2f} mm (< 2 mm, TCP y오프셋 1.6mm 무시분)")
try:
    K.ik(0.6, 0, 0.1, 0)
    check(False, "도달 불가 거부")
except K.IKError:
    check(True, "도달 불가 거부")


# ---- 사라짐
def P(x, y, px, dist=0.5, zone=True):
    return SimpleNamespace(pos=np.array([x, y, 0.0]), px=px, dist=dist, in_zone=zone, box=(px[0] - 20, px[1] - 40, px[0] + 20, px[1] + 40))


def pc(*persons):
    return SimpleNamespace(persons=list(persons), shape=(480, 640))


import copy  # noqa: E402
cfg = copy.deepcopy(C.cfg)
cfg["safety"]["vanish_grace"] = 1.0      # 시험 시간표는 유예 1초·최소 3프레임 기준 (제어판·기본값 변경과 무관하게)
cfg["safety"]["vanish_min_frames"] = 3
v = VanishTracker(cfg)
t = 0.0
for k in range(3):
    v.update(pc(P(0.3, 0, (320, 240))), t + 0.03 * k)
v.update(pc(), t + 0.1)                                  # 가운데에서 사라짐
check(len(v.pending) == 1, "가운데에서 사라지면 대기")
v.update(pc(P(0.31, 0, (322, 240))), t + 0.5)            # 유예 안에 같은 곳 재등장
check(not v.pending and not v.unresolved, "유예 안 재등장 → 복귀")
v.update(pc(P(0.31, 0, (322, 240))), t + 0.53)
v.update(pc(P(0.31, 0, (322, 240))), t + 0.56)
v.update(pc(), t + 0.6)
v.update(pc(P(-0.5, 0.4, (100, 200))), t + 0.8)          # 다른 곳에 다른 사람
new = v.update(pc(P(-0.5, 0.4, (100, 200))), t + 1.8)
check(len(new) == 1 and len(v.unresolved) == 1, "다른 사람 등장으로 해제 안 됨 → 미해결 사라짐")
v.update(pc(P(0.3, 0, (320, 240))), t + 3.0)
check(len(v.unresolved) == 1, "유예 지난 뒤 같은 곳 재등장도 관리자 확인 전엔 유지")
v.clear()
v2 = VanishTracker(cfg)
for k in range(3):
    v2.update(pc(P(0.3, 0, (20, 240))), 0)               # 화면 가장자리
v2.update(pc(), 0.1)
check(not v2.pending, "가장자리에서 사라지면 퇴장")
v3 = VanishTracker(cfg)
for k in range(3):
    v3.update(pc(P(0.3, 0, (320, 240), zone=False)), 0)
v3.update(pc(), 0.1)
check(not v3.pending, "감시 구역 밖에서 사라지면 퇴장")
v4 = VanishTracker(cfg)
for k in range(3):
    v4.update(pc(P(0.3, 0, (320, 240), dist=0.1)), 0)
v4.update(pc(), 0.1)
check(v4.pending_min_dist() == 0.1, "유예 중에는 마지막 거리 유지")
v5 = VanishTracker(cfg)
v5.update(pc(P(0.3, 0, (320, 240))), 0)                  # 1프레임 깜박임
v5.update(pc(), 0.05)
check(not v5.pending, "1프레임 깜박인 오검출은 사라짐 아님")

# ---- 사람-로봇 거리 (3D, 로봇 몸체 점 제외)
from omx.perception.people import link_distance  # noqa: E402
qd = np.array([0, -0.9, 0.6, 1.2, 0, 0])
L = K.fk_points(qd)
a, b = L[3], L[4]                                   # 아래팔 (팔꿈치 → 손목)
axis = (b - a) / np.linalg.norm(b - a)
nrm = np.cross(axis, [0, 0, 1.0]); nrm /= np.linalg.norm(nrm)
mid = (a + b) / 2
rng2 = np.random.default_rng(3)
surf = mid + nrm * 0.020 + rng2.normal(0, 0.003, (150, 3))          # 아래팔 표면 (로봇 자신)
hand = mid + nrm * (0.020 + 0.03) + rng2.normal(0, 0.004, (60, 3))  # 표면에서 3 cm 떨어진 손
d, frac, off = link_distance(np.vstack([surf, hand]), L)
check(0.015 <= d <= 0.045 and off.sum() >= 25, f"팔 표면에 붙은 손(3 cm) 거리 {d * 100:.1f} cm, 로봇 점 비율 {frac:.2f} (손이 지워지지 않음)")
d2, frac2, off2 = link_distance(surf, L)
check(frac2 >= 0.8 and off2.sum() < 25, f"로봇 몸체 점만 있으면 로봇 자기 검출로 판정 (비율 {frac2:.2f})")
small_hand = mid + nrm * 0.05 + rng2.normal(0, 0.004, (30, 3))    # 로봇 점이 대부분인 작은 손
d4, frac4, off4 = link_distance(np.vstack([surf, surf, small_hand]), L)
check(frac4 >= 0.8 and off4.sum() >= 25 and d4 < 0.06, f"로봇 점이 {frac4 * 100:.0f}%여도 손 점이 남으면 사람으로 유지 (거리 {d4 * 100:.1f} cm)")
far = mid + nrm * 0.40 + rng2.normal(0, 0.01, (80, 3))
d3, _, _ = link_distance(far, L)
check(0.35 < d3 < 0.45, f"40 cm 떨어진 사람 거리 {d3:.2f} m")

# ---- 실행기
bus = SimBus()
ex = Executor(bus, C["robot"], C.limits, on_event=lambda *a: None)
ex.start()
ex.set_torque(True)
try:
    ex.set_torque(False) if not ex.at_rest() else None
    check(True, "휴식 자세에서 토크 끄기 허용")
except LimitError:
    check(False, "휴식 자세에서 토크 끄기 허용")
ex.set_torque(True)
for _ in range(10):
    ex.permit(1.0, "test")
    time.sleep(0.02)
m = ex.move_linear(0.16, 0, 0.15, 0.9)
t0 = time.time()
while m.status == "running" and time.time() - t0 < 20:
    ex.permit(1.0, "test")
    time.sleep(0.02)
check(m.status == "done", f"직선 이동 완료 ({m.status})")
try:
    ex.set_torque(False)
    check(False, "휴식 자세 아니면 토크 끄기 거부")
except LimitError:
    check(True, "휴식 자세 아니면 토크 끄기 거부")
for bad in [(0.5, 0, 0.1, 0.0), (0.16, 0, -0.05, 1.5), (0.1, 0.35, 0.1, 1.0)]:
    try:
        ex.move_linear(*bad)
        check(False, f"거부 {bad}")
    except (LimitError, K.IKError) as e:
        check(True, f"거부 {bad}: {e}")
# 허가 끊김 → 멈춤
m = ex.move_linear(0.22, 0.05, 0.10, 1.2)
for _ in range(10):
    ex.permit(1.0, "test")
    time.sleep(0.02)
time.sleep(ex.L["permit_timeout"] + 0.3)                 # 허가를 보내지 않음
q1 = ex.goal.copy()
time.sleep(0.4)
check(np.abs(ex.goal - q1).max() < 1e-6 and m.status == "running", "허가 끊기면 궤적 정지(자리 유지)")
t0 = time.time()
while m.status == "running" and time.time() - t0 < 20:
    ex.permit(1.0, "test")
    time.sleep(0.02)
check(m.status == "done", "허가 복귀 후 이어서 완료")
check(ex.slow_after_stop is False, "완료 후 저속 재개 플래그 해제")
# 조그
start = ex.jog_target = None
tcp0 = K.fk(ex.state.q)
for _ in range(25):
    ex.permit(1.0, "test")
    ex.jog_cmd([0, 0, 1, 0, 0])
    time.sleep(0.04)
time.sleep(0.5)
tcp1 = K.fk(ex.state.q)
check(tcp1[2] - tcp0[2] > 0.01 and abs(tcp1[0] - tcp0[0]) < 0.005, f"조그 z+ {1000 * (tcp1[2] - tcp0[2]):.1f} mm")
g = ex.goal.copy()
time.sleep(0.4)
check(np.abs(ex.goal - g).max() < 1e-6, "키를 떼면(데드맨) 멈춤")
# 관절 조그: 베이스(J1)만 돌고 나머지는 그대로
q0 = ex.state.q.copy()
for _ in range(25):
    ex.permit(1.0, "test")
    ex.jog_cmd([1, 0, 0, 0, 0, 0], mode="joint")
    time.sleep(0.04)
time.sleep(0.5)
dq = ex.state.q - q0
check(dq[0] > 0.1 and np.abs(dq[1:5]).max() < 0.01, f"관절 조그 J1(베이스) +{np.degrees(dq[0]):.1f}°, 다른 관절 그대로")
# 한계까지 밀면 한계에서 멈추고 이유를 알려 준다 (그리퍼 닫기 끝까지)
for _ in range(60):
    ex.permit(1.0, "test")
    ex.jog_cmd([0, 0, 0, 0, 0, -1], mode="joint")
    time.sleep(0.04)
check(ex.goal[5] >= ex.qmin[5] - 1e-9 and "gripper" in ex.jog_blocked, f"관절 한계에서 멈춤: {ex.jog_blocked}")
try:
    ex.jog_cmd([1, 0, 0, 0, 0], mode="joint")
    check(False, "관절 조그 형식 오류 거부")
except LimitError:
    check(True, "관절 조그 형식 오류 거부")
time.sleep(0.4)
# 저장된 자세 보정 (2026-10-12 실기: 저장된 HOME [.., -1.549, .., .., 5.91] 때문에 화재 홈 복귀가 늘 거부됨)
fq = ex.fit_q([0.0322, -1.5493, 1.5493, 1.4021, 5.9135, 0.0537])
check(abs(fq[4] - (5.9135 - 2 * np.pi)) < 1e-9, f"한 바퀴 넘겨 읽힌 손목 5.91 → {fq[4]:.2f} (같은 방향)")
check(abs(fq[1] - ex.qmin[1]) < 1e-9, f"어깨 -1.549 (한계 {ex.qmin[1]:.2f} 를 조금 넘음) → 한계로 당김")
try:
    ex.check_q(fq)
    check(True, "보정한 HOME 은 검사 통과")
except LimitError as e:
    check(False, f"보정한 HOME 은 검사 통과 ({e})")
check(abs(ex.fit_q([0, -2.0, 0, 0, 0, 0.1])[1] - -2.0) < 1e-9, "진짜로 한계 밖(-2.0)은 그대로 → 거부되게")
# 그리퍼 힘: % → 전류(mA)
check(ex.set_gripper_force(50) == C.limits["gripper_current_cap_ma"] // 2 and ex.gripper_force_pct() == 50, "그리퍼 힘 50% = 상한의 절반 전류")
ex.set_gripper_force(C.limits["gripper_current_ma"] * 100 / C.limits["gripper_current_cap_ma"])
# ---- 작업 순서 (PolyScope 방식): 이동은 팔만, 그리퍼는 그리퍼 단계로만
import tempfile, threading  # noqa: E402
from omx.robot.teach import TeachStore, grip_target  # noqa: E402
from omx.robot.task import CycleRunner  # noqa: E402
ts = TeachStore(Path(tempfile.mkdtemp()) / "poses.json")
ts.data["gripper"] = {"open": 0.028, "close": 0.876}          # 실제 설정처럼 닫힘 값이 더 큰 경우
qa = ex.current_q().copy()
qa[5] = 0.6
ts.save_pose("A", qa)
ts.set_home("A")
ts.set_sequence([{"type": "move", "pose": "A", "grip": True, "speed": 30}, {"type": "home", "speed": 300},
                 {"type": "grip", "value": 50}, {"type": "grip", "value": "close"}, {"type": "pause", "value": 0}])
sq = ts.snapshot()["sequence"]
check(sq[0] == {"type": "move", "pose": "A", "speed": 30} and sq[1]["speed"] == 100 and sq[2]["value"] == 50.0
      and sq[4]["value"] == 0.1, f"단계 정리: 옛 grip 키 제거·속도 5~100%·대기 최소 0.1초 {sq}")
check(abs(grip_target(ts.data["gripper"], 50) - 0.452) < 1e-9 and grip_target(ts.data["gripper"], 0) == 0.876,
      "그리퍼 열림 정도 %: 0% = 닫힘 위치, 100% = 열림 위치")
ts.save_grip("부드럽게", 0.3, 20)
try:
    ts.save_grip("열기", 0.3, 20)
    check(False, "기본 동작 이름(열기) 저장 거부")
except ValueError:
    check(True, "기본 동작 이름(열기) 저장 거부")
ts.set_sequence([{"type": "grip", "preset": "부드럽게"}, {"type": "grip", "value": "open", "force": 70}])
check(ts.snapshot()["sequence"] == [{"type": "grip", "preset": "부드럽게"}, {"type": "grip", "value": "open", "force": 70}],
      "그리퍼 단계: 저장한 동작 / 열기+힘")
check(ts.grip_action({"preset": "부드럽게"}) == (0.3, 20, "부드럽게") and ts.grip_action({"value": "close"})[1] == 50,
      "저장한 동작은 저장한 위치·힘, 열기·닫기는 기본 힘")
try:
    ts.delete_grip("부드럽게")
    check(False, "작업에서 쓰는 그리퍼 동작 삭제 거부")
except ValueError as e_:
    check(True, f"작업에서 쓰는 그리퍼 동작 삭제 거부 ({e_})")
try:
    ts.set_sequence([{"type": "grip", "preset": "없는것"}])
    check(False, "없는 그리퍼 동작 단계 거부")
except ValueError:
    check(True, "없는 그리퍼 동작 단계 거부")
ts.set_sequence([{"type": "move", "pose": "A"}, {"type": "home"}, {"type": "grip", "value": 50},
                 {"type": "grip", "value": "close"}, {"type": "pause", "value": 0.1}])
runner = CycleRunner(ex, ts, None, SimpleNamespace(emit=lambda *a, **k: None), C.limits)
feeding = threading.Event()
def _feed():
    while not feeding.is_set():
        ex.permit(1.0, "test")
        time.sleep(0.02)
threading.Thread(target=_feed, daemon=True).start()
g0 = ex.current_q()[5]
runner._step({"type": "move", "pose": "A", "grip": True})
check(abs(ex.current_q()[5] - g0) < 0.05, f"자세 이동은 팔만 (옛 grip 키가 있어도 그리퍼 유지, {ex.current_q()[5]:.2f})")
runner._step({"type": "home"})
check(abs(ex.current_q()[5] - g0) < 0.05, "홈 이동도 팔만")
runner._step({"type": "grip", "value": 50})
check(abs(ex.current_q()[5] - 0.452) < 0.05, f"그리퍼 단계 50% ({ex.current_q()[5]:.2f})")
feeding.set()
# 한 단계만 실행 / 선택한 단계부터
feeding.clear(); threading.Thread(target=_feed, daemon=True).start()
runner.run_one(3)
runner.thread.join(10)
check("완료" in runner.status and abs(ex.current_q()[5] - 0.876) < 0.05, f"한 단계만 실행: {runner.status}")
ts.set_sequence([{"type": "grip", "preset": "부드럽게"}])
runner.run_one(0)
runner.thread.join(10)
check(abs(ex.current_q()[5] - 0.3) < 0.05 and ex.gripper_force_pct() == 20, f"저장한 그리퍼 동작 실행: 위치 {ex.current_q()[5]:.2f}, 힘 {ex.gripper_force_pct()}%")
ts.set_sequence([{"type": "move", "pose": "A"}, {"type": "home"}, {"type": "grip", "value": 50}, {"type": "grip", "value": "close"}])
runner.start(max_cycles=1, start_at=2)
runner.thread.join(20)
check(runner.cycles == 1 and "완료" in runner.status, f"선택한 단계부터 1회: {runner.status}")
feeding.set()


ex.emergency_stop()
try:
    ex.move_joints(ex.current_q())
    check(False, "비상정지 중 명령 거부")
except LimitError:
    check(True, "비상정지 중 명령 거부")
ex.shutdown()
# ---- 보정 클릭: 그리퍼 끝의 깊이 구멍 → 주변에서 가장 앞쪽 깊이를 찾음 (배경 무시)
from omx.perception.calib import Calibration  # noqa: E402
dep = np.full((480, 640), 1500, np.uint16)          # 배경 1.5 m
dep[200:260, 300:340] = 500                         # 그리퍼 0.5 m
dep[225:236, 315:326] = 0                           # 클릭한 곳은 깊이 구멍
fr = SimpleNamespace(depth=dep, depth_scale=0.001)
intr = SimpleNamespace(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
p = Calibration.pixel_to_cam(fr, intr, 320, 230)
check(abs(p[2] - 0.5) < 0.01, f"깊이 구멍 클릭 → 주변 앞쪽 깊이 {p[2]:.3f} m")
p = Calibration.pixel_to_cam(fr, intr, 300, 230)     # 그리퍼 가장자리: 배경과 섞여도 앞쪽
check(abs(p[2] - 0.5) < 0.01, f"가장자리 클릭도 배경(1.5 m) 대신 그리퍼 {p[2]:.3f} m")
try:
    Calibration.pixel_to_cam(SimpleNamespace(depth=np.zeros((480, 640), np.uint16), depth_scale=0.001), intr, 320, 240)
    check(False, "깊이 전혀 없으면 거부")
except ValueError:
    check(True, "깊이 전혀 없으면 거부")

# ---- 사람 3D 점·거리: C++(libomxfast) 과 numpy 기준 구현이 같은 결과
import cv2  # noqa: E402
from omx.perception import geom  # noqa: E402
from omx.perception.people import LINK_RADII, ON_ROBOT_MARGIN, ROBOT_SELF_NEAR  # noqa: E402
check(geom.native_available(), "C++ 거리 계산 라이브러리 사용 (tools/build_native.sh)")
PC = {"depth_range": [0.15, 4.0], "min_depth_pixels": 30, "max_points": 400}
intr_g = SimpleNamespace(fx=605.0, fy=605.0, cx=321.3, cy=239.7)
rng = np.random.default_rng(7)
worst, same_none, n_cases = 0.0, True, 0
for case in range(60):
    mask = np.zeros((480, 640), np.uint8)
    cx_, cy_ = int(rng.integers(80, 560)), int(rng.integers(60, 420))
    cv2.ellipse(mask, (cx_, cy_), (int(rng.integers(3, 140)), int(rng.integers(3, 200))), 0, 0, 360, 1, -1)
    dep = (rng.normal(rng.uniform(0.3, 1.5), 0.02, (480, 640)) / 0.001).clip(0, 65535).astype(np.uint16)
    dep[rng.random((480, 640)) < 0.15] = 0                                   # 깊이 구멍
    bg = rng.random((480, 640)) < 0.2
    dep[bg] = (rng.uniform(2.0, 3.5) / 0.001)                                # 배경 섞임
    q = np.r_[rng.uniform(-1, 1, 5), 0.0]
    Lk = K.fk_points(q)
    th = rng.uniform(-0.3, 0.3)
    Tg = np.eye(4)
    Tg[:3, :3] = [[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]]
    Tg[:3, 3] = rng.uniform(-0.5, 0.5, 3)
    if case % 3 == 0:                                                         # 손이 팔에 닿은 경우: 점을 링크 근처로
        Tg = np.eye(4); Tg[:3, 3] = Lk[3] - np.array([0, 0, dep[dep > 0].mean() * 0.001])
    for T_, L_ in ((None, None), (Tg, None), (Tg, Lk)):
        a_ = geom.person_geometry(mask, dep, 0.001, intr_g, T_, L_, LINK_RADII, PC, ON_ROBOT_MARGIN, ROBOT_SELF_NEAR)
        b_ = geom.person_geometry_py(mask, dep, 0.001, intr_g, T_, L_, LINK_RADII, PC, ON_ROBOT_MARGIN, ROBOT_SELF_NEAR)
        n_cases += 1
        if (a_ is None) != (b_ is None):
            same_none = False
            continue
        if a_ is None:
            continue
        for k_ in ("npts", "n_off"):
            if a_[k_] != b_[k_]:
                worst = max(worst, 1.0)
        for k_ in ("cam_dist", "dist", "on_frac", "near_frac"):
            worst = max(worst, abs(a_[k_] - b_[k_]) if np.isfinite(b_[k_]) else float(a_[k_] != b_[k_]))
        worst = max(worst, float(np.abs(a_["pos"] - b_["pos"]).max()))
check(same_none, f"깊이 부족 판정 일치 ({n_cases}가지)")
check(worst < 1e-9, f"C++ ↔ numpy 최대 차이 {worst:.2e} (점 수·거리·비율·위치)")
# 점이 전부 로봇 몸체 위면 거리 0 이고 대표 위치가 있다(예전: 위치 NaN → 감시 구역 밖으로 빠져 무시됨)
Lk = K.fk_points(np.array([0, -0.6, 0.3, 1.2, 0, 0]))
mask = np.zeros((480, 640), np.uint8); mask[230:250, 310:330] = 1
dep = np.full((480, 640), 500, np.uint16)
Tg = np.eye(4); Tg[:3, 3] = Lk[3] - np.array([(320 - intr_g.cx) * 0.5 / intr_g.fx, (240 - intr_g.cy) * 0.5 / intr_g.fy, 0.5])
g_ = geom.person_geometry(mask, dep, 0.001, intr_g, Tg, Lk, LINK_RADII, PC, ON_ROBOT_MARGIN, ROBOT_SELF_NEAR)
check(g_["dist"] == 0.0 and np.isfinite(g_["pos"]).all(), f"점이 전부 몸체 위: 거리 0, 위치 {np.round(g_['pos'], 3)}")

# ---- 이벤트 로그 정책: 정해진 종류만 DB 에 (감속·토크 등은 기록 파일만)
from omx.events import EventHub  # noqa: E402
added = []
hub = EventHub(C.cfg, SimpleNamespace(add=lambda t, *a, **k: added.append(t) or len(added)))
for t_ in ("human_slow", "robot", "cycle", "calib_auto", "restart", "cycle_stop", "robot_temp", "robot_angle", "clip_save", "human_stop"):
    hub.emit(t_, "시험 " + t_)
check(added == ["restart", "cycle_stop", "robot_temp", "clip_save", "human_stop"], f"로그에 남은 종류 {added} (관절 각도 이상은 빠짐)")

# ---- 화면 경보 표시값: 사람이 정지 거리 안 → human_alarm, 재가동하면 꺼짐
from omx.perception.people import Percept  # noqa: E402
from omx.safety.manager import SafetyManager  # noqa: E402
fake_ex = SimpleNamespace(fault=None, estop=False, permit=lambda *a: None, stop_motion=lambda *a: None, emergency_stop=lambda: None)
fake_io = SimpleNamespace(fire=lambda: False, healthy=lambda: True, health_msg=lambda: "", set_door=lambda *a: None)
fake_per = SimpleNamespace(latest=None, cam=SimpleNamespace(error=""))
sm = SafetyManager(C, fake_ex, fake_per, fake_io, SimpleNamespace(emit=lambda *a, **k: None))
seq = [0]
def feed(d, n):
    for _ in range(n):
        seq[0] += 1
        fake_per.latest = (None, Percept(time.monotonic(), seq[0], min_dist=d))
        sm._step()
feed(5.0, 5)
sm.restart()
check(not sm.snapshot()["human_alarm"], "사람 없으면 화면 경보 꺼짐")
feed(sm.profile()["stop"] * 0.5, 4)
check(sm.snapshot()["human_alarm"] and sm.stop_latched, f"정지 거리 안 → 화면 경보 켜짐 ({sm.stop_latched})")
feed(5.0, 5)
check(sm.snapshot()["human_alarm"], "사람이 떠나도 재가동 전까지 경보 유지")
ok, probs = sm.restart()
check(ok and not sm.snapshot()["human_alarm"], f"재가동하면 경보 꺼짐 {probs}")

# ---- 티칭 모드: 사람이 가까워도 감속·정지·사라짐 없음, 작업 실행 차단, 끄면 다시 적용
permits = []
fake_ex.permit = lambda sc, why: permits.append((sc, why))
emitted = []
sm.events = SimpleNamespace(emit=lambda t, *a, **k: emitted.append(t))
feed(sm.profile()["stop"] * 0.5, 4)
check(sm.state == "STOP" and sm.stop_latched.startswith("사람 근접"), "티칭 모드 전: 사람 근접 → 정지")
sm.set_teach_mode(True, "admin")
check(not sm.stop_latched and sm.zone == "CLEAR" and "teach_mode" in emitted, "티칭 모드 켜면 사람 때문에 걸린 정지 해제 + 기록")
emitted.clear()
feed(sm.profile()["stop"] * 0.3, 6)
check(sm.state == "NORMAL" and permits[-1][0] == 1.0 and not emitted, f"티칭 모드: 사람이 정지 거리 안이어도 허가 1.0, 이벤트 없음 ({sm.state}, {permits[-1]})")
sm.last_percept_t -= 10
sm._step()
check(not sm.stop_latched, "티칭 모드: 인식 멈춤으로 정지하지 않음")
fake_ex.estop = True
sm._step()
check(sm.stop_latched == "비상정지 버튼", "티칭 모드에서도 비상정지는 그대로")
fake_ex.estop = False
ok, probs = sm.restart()
check(ok, f"티칭 모드: 사람이 가까워도 재가동 가능 {probs}")
from omx.robot.task import CycleRunner  # noqa: E402
cr = CycleRunner.__new__(CycleRunner)
cr.thread, cr.safety = None, sm
cr.teach = SimpleNamespace(snapshot=lambda: {"sequence": [{"type": "pause", "value": 1}]})
try:
    cr._check_start()
    check(False, "티칭 모드에서 작업 실행 거부")
except RuntimeError as e:
    check("티칭 모드" in str(e), f"티칭 모드에서 작업 실행 거부 ({e})")
sm.set_teach_mode(False, "admin")
feed(sm.profile()["stop"] * 0.5, 4)
check(sm.state == "STOP" and sm.stop_latched.startswith("사람 근접"), "티칭 모드 끄면 사람 근접 정지 다시 적용")

print("\n실패:", fails if fails else "없음")
sys.exit(1 if fails else 0)
