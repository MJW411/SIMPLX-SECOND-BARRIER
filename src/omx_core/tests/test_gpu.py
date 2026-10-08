"""GPU 앞뒤 처리(omx/native) 결과가 기존 numpy 경로와 같은지, 그리고 CPU 시간이 얼마나 줄었는지.

python tests/test_gpu.py [영상.mp4]   (기본: data/clips 의 사람 근접 정지 클립)
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import omx  # noqa: E402,F401  (수치 라이브러리 스레드 제한)
from omx.perception.gpu_post import blocking_sync  # noqa: E402

blocking_sync()
import cv2  # noqa: E402
import numpy as np  # noqa: E402

from omx.config import MODELS_DIR  # noqa: E402
from omx.perception.detector import det_mask  # noqa: E402
from omx.perception.trt_direct import TRTYolo  # noqa: E402

fails = []


def check(c, what):
    print(("  OK   " if c else "  FAIL ") + what)
    if not c:
        fails.append(what)


clip = sys.argv[1] if len(sys.argv) > 1 else str(sorted(Path("data/clips").glob("*human_stop*.mp4"), key=lambda p: -p.stat().st_size)[0])
cap = cv2.VideoCapture(clip)
frames = []
while len(frames) < 120:
    ok, f = cap.read()
    if not ok:
        break
    frames.append(f)
eng = MODELS_DIR / "yolo11n-seg.engine"
g = TRTYolo(str(eng), conf=0.3)
c = TRTYolo(str(eng), conf=0.3, use_gpu_post=False)
check(g.gpu is not None, "GPU 앞뒤 처리 라이브러리 사용")
nd = worst_box = worst_conf = 0.0
ious, mism = [], 0
for f in frames:
    a, b = g.infer(f, classes=[0]), c.infer(f, classes=[0])
    if len(a) != len(b):
        mism += 1
        continue
    for x, y in zip(sorted(a, key=lambda d: -d.conf), sorted(b, key=lambda d: -d.conf)):
        nd += 1
        worst_box = max(worst_box, float(np.abs(np.array(x.box) - np.array(y.box)).max()))
        worst_conf = max(worst_conf, abs(x.conf - y.conf))
        mx, my = det_mask(x, f.shape), det_mask(y, f.shape)
        ious.append((mx & my).sum() / max(1, (mx | my).sum()))
print(f"  프레임 {len(frames)}, 검출 {int(nd)}")
check(mism <= len(frames) * 0.03, f"검출 수가 다른 프레임 {mism}개 (경계 점수 차이, 3% 이하)")
check(worst_box < 4.0, f"박스 최대 차이 {worst_box:.2f} px")
check(worst_conf < 0.03, f"확신도 최대 차이 {worst_conf:.3f}")
check(not ious or np.min(ious) > 0.85, f"마스크 IoU 최소 {np.min(ious) if ious else 1:.3f} · 평균 {np.mean(ious) if ious else 1:.3f}")


def cost(m):
    for f in frames[:10]:
        m.infer(f, classes=[0])
    w0, c0 = time.perf_counter(), time.process_time()
    for f in frames:
        m.infer(f, classes=[0])
    n = len(frames)
    return (time.perf_counter() - w0) / n * 1000, (time.process_time() - c0) / n * 1000


gw, gc = cost(g)
cw, cc = cost(c)
print(f"  프레임당  GPU 경로: 걸린 시간 {gw:.1f} ms · CPU {gc:.1f} ms  |  CPU 경로: 걸린 시간 {cw:.1f} ms · CPU {cc:.1f} ms")
check(gc < cc, "GPU 경로가 CPU 시간을 덜 씀")
print("\n실패:", fails if fails else "없음")
sys.exit(1 if fails else 0)
