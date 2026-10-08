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
    """lanes: 같은 모델을 쓰는 독립 실행 통로 수. 사람(0)·헬멧(1) 카메라가 엔진 하나를 같이 쓰되
    각자 실행 컨텍스트를 가져 서로 기다리지 않는다 (가중치·CUDA 컨텍스트는 한 번만 올라간다).
    require_any: 모델 이름표에 이 중 하나라도 있어야 쓴다 (예: human/person). 없으면 교체를 거부한다."""

    def __init__(self, data_dir, model_name, conf=0.3, imgsz=640, backend="direct", lanes=1, require_any=None):
        self.backend = backend   # direct: .engine 을 torch 없이 TensorRT 로 직접 (메모리 약 1 GB 절약)
        self.data_dir = Path(data_dir)
        self.up_dir = self.data_dir / "models"
        self.eng_dir = self.data_dir / "engines"
        self.up_dir.mkdir(parents=True, exist_ok=True)
        self.eng_dir.mkdir(parents=True, exist_ok=True)
        self.conf = conf
        self.imgsz = imgsz
        self.n_lanes = max(1, int(lanes))
        self.require_any = set(require_any or ())
        self.models = [None] * self.n_lanes
        self.lane_locks = [threading.Lock() for _ in range(self.n_lanes)]
        self.model_name = ""
        self.model_path = None
        self.task = ""
        self.names = {}
        self.status = "불러오기 전"
        self.convert_status = ""
        self.infer_ms = 0.0
        self.lane_ms = [0.0] * self.n_lanes
        self.load_error = ""
        self.lock = threading.Lock()
        try:
            self.select(model_name, wait=True)
        except Exception as e:           # 모델이 없거나 이름이 잘못돼도 프로세스는 살아 있고, ready=False 로 정지를 건다
            self.status = self.load_error = f"불러오기 실패: {model_name}: {e}"
            log.error("model select %s: %s", model_name, e)

    @property
    def model(self):
        return self.models[0]

    @property
    def ready(self):
        """모델이 올라와 시험 추론까지 성공했는가. 아니면 사람을 볼 수 없으므로 안전 관리자가 고장 정지한다."""
        return self.models[0] is not None

    def class_ids(self, wanted):
        """이름 목록 → 이 모델의 클래스 번호 목록 (없는 이름은 뺀다)."""
        inv = {v: k for k, v in self.names.items()}
        return [inv[n] for n in wanted if n in inv]

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
        name = str(name)
        if not name or Path(name).name != name or Path(name).suffix not in EXTS:
            raise ValueError(f"모델 파일 이름이 잘못됨: {name!r}")      # 경로(../, /) 차단
        for d in (self.eng_dir, self.up_dir, MODELS_DIR):
            p = d / name
            if p.is_file():
                return p
        raise FileNotFoundError(name)

    def _cached_engine(self, pt):
        h = hashlib.md5(pt.read_bytes()).hexdigest()[:8]
        return self.eng_dir / f"{pt.stem}_{h}_fp16.engine", h

    def select(self, name, wait=False):
        """모델 교체. wait=True 면 끝날 때까지 기다리고 성공 여부(bool)를 돌려준다.
        실패하면 지금 쓰던 모델을 그대로 쓴다."""
        path = self.resolve(name)
        if path.suffix == ".pt":
            eng, _ = self._cached_engine(path)
            if eng.exists():
                path = eng
            else:
                self._convert_async(path, eng)
        result = {}
        t = threading.Thread(target=lambda: result.setdefault("ok", self._load(path)), daemon=True)
        t.start()
        if wait:
            t.join()
            return bool(result.get("ok"))
        return None

    def _check_names(self, names):
        if self.require_any and not (self.require_any & set(names.values())):
            raise ValueError(f"필요한 클래스({', '.join(sorted(self.require_any))})가 없는 모델: {sorted(names.values())}")

    def _install(self, lanes, path, task, names):
        """새 모델로 바꾸고 옛 모델의 GPU 자원을 푼다 (각 레인의 진행 중인 추론이 끝난 뒤)."""
        old = []
        for i in range(self.n_lanes):
            with self.lane_locks[i]:
                old.append(self.models[i])
                self.models[i] = lanes[i]
        with self.lock:
            self.model_path, self.model_name, self.task, self.names = path, path.name, task, names
        for m in old:
            if m is not None and hasattr(m, "close") and m not in lanes:
                try:
                    m.close()
                except Exception:
                    log.exception("old model close")

    def _load(self, path):
        self.status = f"불러오는 중: {path.name}"
        if path.suffix == ".engine" and self.backend == "direct":
            m = None
            try:
                from .trt_direct import TRTYolo
                m = TRTYolo(str(path), conf=self.conf)
                self._check_names(m.names)
                m.infer(np.zeros((480, 640, 3), np.uint8))
                lanes = [m] + [TRTYolo(conf=self.conf, share=m) for _ in range(self.n_lanes - 1)]
                for x in lanes[1:]:
                    x.infer(np.zeros((480, 640, 3), np.uint8), masks=False)
                self._install(lanes, path, m.task, m.names)
                self.status = f"사용 중: {path.name} ({m.task}, TensorRT 직접 실행)"
                self.load_error = ""
                log.info("model loaded (direct TRT) %s task=%s lanes=%d", path, m.task, self.n_lanes)
                return True
            except ValueError as e:                       # 클래스가 맞지 않는 모델: ultralytics 로도 같으니 바로 실패
                if m is not None:
                    m.close()
                self.status = self.load_error = f"불러오기 실패: {path.name}: {e}"
                log.error("model rejected %s: %s", path, e)
                return False
            except Exception as e:
                log.warning("TensorRT 직접 로드 실패, ultralytics로 대체: %s", e)
        try:
            from ultralytics import YOLO
            task = None
            if path.suffix == ".engine":
                # 엔진 앞 메타데이터의 task (없으면 이름으로 추정)
                from .trt_direct import read_meta
                task = read_meta(str(path))[0].get("task") or ("segment" if "seg" in path.name else "detect")
            m = YOLO(str(path), task=task)
            self._check_names(m.names)
            m.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.imgsz, verbose=False)
            self._install([m] * self.n_lanes, path, m.task, m.names)
            self.status = f"사용 중: {path.name} ({m.task})"
            self.load_error = ""
            log.info("model loaded %s task=%s", path, m.task)
            return True
        except Exception as e:
            self.status = self.load_error = f"불러오기 실패: {path.name}: {e}"
            log.exception("model load")
            return False

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
        tmp = p.with_name(p.name + ".part")
        tmp.write_bytes(data)
        tmp.replace(p)
        return name

    # ---- 추론 ----
    def infer(self, img, classes=None, lane=0, masks=True):
        """lane: 실행 통로 번호 (카메라별). masks=False 면 마스크를 만들지 않는다."""
        m0 = self.models[lane]
        # ultralytics 대체 경로는 레인들이 모델 객체 하나를 같이 쓰므로 잠금도 하나로
        lock = self.lane_locks[0] if (m0 is not None and hasattr(m0, "predict")) else self.lane_locks[lane]
        with lock:
            m = self.models[lane]
            if m is None:
                return []
            if not hasattr(m, "predict"):                     # TensorRT 직접 실행기
                m.conf = self.conf
                dets = m.infer(img, classes, masks)
                ms = m.infer_ms
            else:
                t0 = time.monotonic()
                r = m.predict(img, imgsz=self.imgsz, conf=self.conf, classes=classes, verbose=False)[0]
                ms = (time.monotonic() - t0) * 1000
                dets = []
                if r.boxes is not None:
                    boxes = r.boxes.xyxy.cpu().numpy()
                    cls = r.boxes.cls.cpu().numpy().astype(int)
                    conf = r.boxes.conf.cpu().numpy()
                    polys = r.masks.xy if (masks and r.masks is not None) else [None] * len(boxes)
                    for b, c, s, p in zip(boxes, cls, conf, polys):
                        dets.append(Detection(int(c), self.names.get(int(c), str(c)), float(s), tuple(b.tolist()),
                                              None if p is None or len(p) < 3 else p.astype(np.int32)))
        self.lane_ms[lane] = ms
        if lane == 0:
            self.infer_ms = ms
        return dets

    def gpu_post_error(self):
        m = self.models[0]
        return getattr(m, "gpu_error", "") if m is not None else ""


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
