"""메모리·속도 비교: ultralytics(.engine) vs TensorRT 직접 실행. 각자 별도 프로세스로 잰다.
  python tools/bench_trt_direct.py [엔진 경로]"""
import json, subprocess, sys
from pathlib import Path

ENG = sys.argv[1] if len(sys.argv) > 1 else str(Path.home() / "models/yolo11n-seg.engine")
ROOT = Path(__file__).resolve().parent.parent
CODE = r'''
import sys, time, json, psutil, os, cv2, numpy as np
sys.path.insert(0, "%(root)s")
vm0 = psutil.virtual_memory()
img = cv2.imread("%(img)s")
mode = "%(mode)s"
t0 = time.time()
if mode == "ultralytics":
    from ultralytics import YOLO
    m = YOLO("%(eng)s", task="segment")
    run = lambda: m.predict(img, imgsz=640, conf=0.3, classes=[0], verbose=False)[0]
    count = lambda r: len(r.boxes)
    confs = lambda r: sorted([round(float(c), 2) for c in r.boxes.conf], reverse=True)
else:
    from omx.perception.trt_direct import TRTYolo
    m = TRTYolo("%(eng)s", conf=0.3)
    run = lambda: m.infer(img, classes=[0])
    count = len
    confs = lambda r: sorted([round(d.conf, 2) for d in r], reverse=True)
load_s = time.time() - t0
for _ in range(10): r = run()
ts = []
for _ in range(100):
    t = time.time(); r = run(); ts.append((time.time() - t) * 1000)
p = psutil.Process()
vm1 = psutil.virtual_memory()
print(json.dumps({"mode": mode, "load_s": round(load_s, 1), "ms_median": round(float(np.median(ts)), 1),
  "ms_p95": round(float(np.percentile(ts, 95)), 1), "rss_mb": round(p.memory_info().rss / 1e6),
  "sys_ram_delta_mb": round(((vm0.available - vm1.available)) / 1e6), "persons": count(r), "confs": confs(r)}))
'''
import ultralytics
img = str(Path(ultralytics.__file__).parent / "assets" / "bus.jpg")
res = []
for mode in ("ultralytics", "direct"):
    out = subprocess.run([sys.executable, "-c", CODE % {"root": ROOT, "img": img, "mode": mode, "eng": ENG}],
                         capture_output=True, text=True, env={k: v for k, v in __import__("os").environ.items() if k != "PYTHONPATH"})
    line = [l for l in out.stdout.splitlines() if l.startswith("{")]
    res.append(json.loads(line[-1]) if line else {"mode": mode, "error": out.stderr[-800:]})
    print(res[-1], flush=True)
