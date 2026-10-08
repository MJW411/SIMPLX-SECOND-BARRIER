"""v2 통합 모델(fall-down / helmet / human) 시험: 여러 클래스 GPU 경로 = CPU 경로, 엔진 공유 레인, 모델 검증.

  python tests/test_v2.py [영상.mp4]
"""
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import omx  # noqa: E402,F401
from omx.perception.gpu_post import blocking_sync  # noqa: E402

blocking_sync()
import cv2  # noqa: E402
import numpy as np  # noqa: E402

from omx.config import MODELS_DIR  # noqa: E402
from omx.perception.detector import Detector  # noqa: E402
from omx.perception.trt_direct import TRTYolo  # noqa: E402

fails = []


def check(c, what):
    print(("  OK   " if c else "  FAIL ") + what)
    if not c:
        fails.append(what)


ENG = MODELS_DIR / "v2_best.engine"
if not ENG.exists():
    print(f"건너뜀: {ENG} 없음")
    sys.exit(0)

clip = sys.argv[1] if len(sys.argv) > 1 else None
frames = []
if clip:
    cap = cv2.VideoCapture(clip)
    while len(frames) < 90:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
if not frames:                                  # 영상이 없으면 잡음 영상 (경로 일치 확인용)
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 255, (480, 640, 3), np.uint8) for _ in range(10)]

print("[GPU 여러 클래스 = CPU]")
g = TRTYolo(str(ENG), conf=0.25)
c = TRTYolo(str(ENG), conf=0.25, use_gpu_post=False)
names = {v: k for k, v in g.names.items()}
check({"human", "fall-down", "helmet"} <= set(names), f"클래스 이름표: {g.names}")
people = [names["human"], names["fall-down"]]
check(g.gpu is not None, "GPU 앞뒤 처리 사용")
nd = mism = 0
worst_box = worst_conf = 0.0
for f in frames:
    for cls in (people, None):
        a, b = g.infer(f, cls), c.infer(f, cls)
        nd += len(a)
        if len(a) != len(b):
            mism += 1
            continue
        for x, y in zip(sorted(a, key=lambda d: d.box), sorted(b, key=lambda d: d.box)):
            if x.cls != y.cls:
                mism += 1
            worst_box = max(worst_box, float(np.abs(np.array(x.box) - np.array(y.box)).max()))
            worst_conf = max(worst_conf, abs(x.conf - y.conf))
print(f"  검출 {nd}개 / 프레임 {len(frames)}")
check(mism <= max(1, 0.03 * len(frames)), f"검출 수·클래스가 다른 경우 {mism}건 (3% 이하)")
check(worst_box < 1.0 and worst_conf < 0.01, f"박스 최대 차이 {worst_box:.2f}px, 확신도 {worst_conf:.4f}")
a = g.infer(frames[0], people, masks=False)
check(all(d.poly is None for d in a), "masks=False 면 마스크를 만들지 않음")
check(all(d.cls in people for f in frames[:10] for d in g.infer(f, people)), "고른 클래스만 나옴")

print("[엔진 공유 레인]")
s = TRTYolo(conf=0.25, share=g)
check(s.engine is g.engine, "가중치(엔진)는 같은 객체")
r1, r2 = g.infer(frames[0], people), s.infer(frames[0], people)
check(len(r1) == len(r2) and all(abs(x.conf - y.conf) < 1e-4 for x, y in zip(r1, r2)), "두 레인 결과 같음")
s.close()
c.close()
check(g.infer(frames[0], people) is not None, "다른 레인을 닫아도 첫 레인은 계속 동작")

print("[Detector: 레인·검증·준비 상태]")
d = tempfile.mkdtemp()
det = Detector(d, "v2_best.engine", 0.5, lanes=2, require_any=["human", "person"])
check(det.ready and det.model_name == "v2_best.engine", f"불러옴: {det.status}")
check(det.class_ids(["human", "fall-down", "없음"]) == [names["human"], names["fall-down"]], "이름 → 번호")
det.infer(frames[0], people, lane=0)
det.infer(frames[0], None, lane=1, masks=False)
check(det.lane_ms[0] > 0 and det.lane_ms[1] > 0, f"레인별 추론 시간 {det.lane_ms}")
helmet_only = MODELS_DIR / "helmet_best_fixed.engine"
if helmet_only.exists():
    ok = det.select(helmet_only.name, wait=True)
    check(ok is False and det.model_name == "v2_best.engine" and det.ready,
          "사람 클래스 없는 모델로 교체 거부, 원래 모델 유지")
for bad in ("../etc/passwd", "/home/user/models/v2_best.engine", "x.txt"):
    try:
        det.select(bad, wait=True)
        check(False, f"잘못된 이름 거부: {bad}")
    except ValueError:
        check(True, f"잘못된 이름 거부: {bad}")
missing = Detector(d, "없는모델.engine", 0.5, lanes=2, require_any=["human"])
check(not missing.ready and missing.infer(frames[0]) == [] and "실패" in missing.status,
      "없는 모델로 시작 → 준비 안 됨(ready=False), 프로세스는 계속")

t0 = time.monotonic()
for f in frames[:30]:
    det.infer(f, people, lane=0)
print(f"  사람 레인 평균 {1000 * (time.monotonic() - t0) / min(30, len(frames)):.1f} ms/프레임")

print()
print("실패 %d건" % len(fails) if fails else "모두 통과")
sys.exit(1 if fails else 0)
