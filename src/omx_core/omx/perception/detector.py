"""YOLO 검출기 래퍼 + 가중치 교체.

- 가중치 목록: ~/models(읽기 전용), data/models(웹 업로드), data/engines(변환 캐시)
- .pt를 고르면 즉시 .pt로 돌리면서, 백그라운드에서 TensorRT FP16 엔진으로 변환해
  data/engines/에 캐시하고 끝나면 엔진으로 자동 교체한다. (~/models에는 쓰지 않는다)
- 세그멘테이션 모델이면 사람 마스크(다각형 → 래스터), 검출 모델이면 박스를 준다.
"""
import hashlib
import logging
import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from ..config import MODELS_DIR, ROOT

log = logging.getLogger("detector")
EXTS = (".pt", ".engine", ".onnx")


@dataclass
class Detection:
    cls: int
    name: str
    conf: float
    box: tuple                  # x1,y1,x2,y2 (px)
    poly: np.ndarray = None     # 마스크 외곽선 (N,2) px, 세그멘테이션일 때


class Detector:
    def __init__(self, data_dir, model_name, conf=0.3, imgsz=640, backend="direct"):
        self.backend = backend   # direct: .engine 을 torch 없이 TensorRT 로 직접 (메모리 약 1 GB 절약)
        self.data_dir = Path(data_dir)
        self.up_dir = self.data_dir / "models"
        self.eng_dir = self.data_dir / "engines"
        self.up_dir.mkdir(parents=True, exist_ok=True)
        self.eng_dir.mkdir(parents=True, exist_ok=True)
        self.conf = conf
        self.imgsz = imgsz
        self.model = None
        self.model_name = ""
        self.model_path = None
        self.task = ""
        self.names = {}
        self.status = "불러오기 전"
        self.convert_status = ""
        self.infer_ms = 0.0
        self.lock = threading.Lock()
        self.select(model_name, wait=True)

    # ---- 목록·선택 ----
    def list_models(self):
        out = []
        for d, tag in ((MODELS_DIR, "models"), (self.up_dir, "upload"), (self.eng_dir, "cache")):
            if d.exists():
                for p in sorted(d.iterdir()):
                    if p.suffix in EXTS and p.is_file():
                        out.append({"name": p.name, "dir": tag, "size_mb": round(p.stat().st_size / 1e6, 1)})
        return out

    def resolve(self, name):
        for d in (self.eng_dir, self.up_dir, MODELS_DIR):
            p = d / name
            if p.exists():
                return p
        raise FileNotFoundError(name)

    def _cached_engine(self, pt):
        h = hashlib.md5(pt.read_bytes()).hexdigest()[:8]
        return self.eng_dir / f"{pt.stem}_{h}_fp16.engine", h

    def select(self, name, wait=False):
        path = self.resolve(name)
        if path.suffix == ".pt":
            eng, _ = self._cached_engine(path)
            if eng.exists():
                path = eng
            else:
                self._convert_async(path, eng)
        t = threading.Thread(target=self._load, args=(path,), daemon=True)
        t.start()
        if wait:
            t.join()

    def _load(self, path):
        self.status = f"불러오는 중: {path.name}"
        if path.suffix == ".engine" and self.backend == "direct":
            try:
                from .trt_direct import TRTYolo
                m = TRTYolo(str(path), conf=self.conf)
                m.infer(np.zeros((480, 640, 3), np.uint8))
                with self.lock:
                    self.model, self.model_path, self.model_name = m, path, path.name
                    self.task, self.names = m.task, m.names
                self.status = f"사용 중: {path.name} ({m.task}, TensorRT 직접 실행)"
                log.info("model loaded (direct TRT) %s task=%s", path, m.task)
                return
            except Exception as e:
                log.warning("TensorRT 직접 로드 실패, ultralytics로 대체: %s", e)
        from ultralytics import YOLO
        try:
            task = None
            if path.suffix == ".engine":
                # 엔진은 task를 알 수 없을 때가 있어 이름으로 추정
                task = "segment" if "seg" in path.name else "detect"
            m = YOLO(str(path), task=task)
            m.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.imgsz, verbose=False)
            with self.lock:
                self.model = m
                self.model_path = path
                self.model_name = path.name
                self.task = m.task
                self.names = m.names
            self.status = f"사용 중: {path.name} ({m.task})"
            log.info("model loaded %s task=%s", path, m.task)
        except Exception as e:
            self.status = f"불러오기 실패: {path.name}: {e}"
            log.exception("model load")

    def _convert_async(self, pt, eng):
        def run():
            self.convert_status = f"TensorRT 변환 중: {pt.name} (몇 분 걸림)"
            work = self.eng_dir / f"_work_{pt.stem}"
            work.mkdir(exist_ok=True)
            src = work / pt.name
            shutil.copy2(pt, src)
            code = ("from ultralytics import YOLO;"
                    f"YOLO(r'{src}').export(format='engine', half=True, imgsz={self.imgsz}, workspace=2)")
            env = dict(os.environ)
            env.pop("PYTHONPATH", None)
            t0 = time.time()
            r = subprocess.run([sys.executable, "-c", code], cwd=work, env=env,
                               capture_output=True, text=True)
            out = src.with_suffix(".engine")
            if r.returncode == 0 and out.exists():
                out.replace(eng)
                self.convert_status = f"변환 완료: {eng.name} ({time.time() - t0:.0f}s)"
                if self.model_name == pt.name:
                    self._load(eng)
            else:
                self.convert_status = f"변환 실패: {pt.name}: {r.stderr[-300:]}"
            shutil.rmtree(work, ignore_errors=True)
        threading.Thread(target=run, daemon=True, name="trt-convert").start()

    def save_upload(self, filename, data):
        name = Path(filename).name
        if Path(name).suffix not in EXTS:
            raise ValueError("지원 형식: .pt .engine .onnx")
        p = self.up_dir / name
        p.write_bytes(data)
        return name

    # ---- 추론 ----
    def infer(self, img, classes=None):
        with self.lock:
            m = self.model
        if m is None:
            return []
        if not hasattr(m, "predict"):                     # TensorRT 직접 실행기
            m.conf = self.conf
            dets = m.infer(img, classes)
            self.infer_ms = m.infer_ms
            return dets
        t0 = time.monotonic()
        r = m.predict(img, imgsz=self.imgsz, conf=self.conf, classes=classes, verbose=False)[0]
        self.infer_ms = (time.monotonic() - t0) * 1000
        dets = []
        if r.boxes is None:
            return dets
        boxes = r.boxes.xyxy.cpu().numpy()
        cls = r.boxes.cls.cpu().numpy().astype(int)
        conf = r.boxes.conf.cpu().numpy()
        polys = r.masks.xy if r.masks is not None else [None] * len(boxes)
        for b, c, s, p in zip(boxes, cls, conf, polys):
            dets.append(Detection(int(c), self.names.get(int(c), str(c)), float(s), tuple(b.tolist()),
                                  None if p is None or len(p) < 3 else p.astype(np.int32)))
        return dets


def det_mask(det, shape):
    """검출 → 불리언 마스크. 다각형이 없으면 박스 가운데 60% 영역."""
    m = np.zeros(shape[:2], np.uint8)
    if det.poly is not None:
        cv2.fillPoly(m, [det.poly], 1)
    else:
        x1, y1, x2, y2 = det.box
        w, h = x2 - x1, y2 - y1
        cv2.rectangle(m, (int(x1 + 0.2 * w), int(y1 + 0.2 * h)), (int(x2 - 0.2 * w), int(y2 - 0.2 * h)), 1, -1)
    return m.astype(bool)
