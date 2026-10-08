"""torch·ultralytics 없이 TensorRT 엔진을 직접 돌리는 YOLO(seg/detect) 추론기 — 메모리 절약용.

- CUDA는 ctypes로 libcudart만 부른다(추가 설치 없음).
- ultralytics가 만든 .engine 앞의 메타데이터(4바이트 길이 + JSON)를 읽어 클래스 이름·입력 크기를 얻는다.
- 앞뒤 처리는 GPU 라이브러리(omx/native, gpu_post.py)가 있으면 GPU에서 한다: 원본 영상만 올리고 사람 후보·마스크만
  내려받는다. 없으면 numpy/OpenCV 경로(아래 _infer_host)로 같은 결과를 낸다.
Detector 와 같은 infer() 결과(Detection 목록)를 돌려준다.
"""
import ctypes
import json
import logging
import time

import cv2
import numpy as np

log = logging.getLogger("trt")
_cud = None


def _cudart():
    global _cud
    if _cud is None:
        _cud = ctypes.CDLL("libcudart.so")
    return _cud


def _ck(r, what):
    if r != 0:
        raise RuntimeError(f"CUDA 오류 {r} ({what})")


class TRTYolo:
    def __init__(self, path, conf=0.3, iou=0.5, use_gpu_post=True):
        from .gpu_post import GpuPost, blocking_sync
        blocking_sync()          # 이미 CUDA 를 쓰기 시작했으면 효과 없음 (main 에서 먼저 부른다)
        import tensorrt as trt
        self.conf, self.iou = conf, iou
        raw = open(path, "rb").read()
        self.meta = {}
        n = int.from_bytes(raw[:4], "little")
        if 0 < n < 100000 and raw[4:5] == b"{":
            try:
                self.meta = json.loads(raw[4:4 + n].decode())
                raw = raw[4 + n:]
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
        names = self.meta.get("names", {})
        self.names = {int(k): v for k, v in names.items()} if isinstance(names, dict) else dict(enumerate(names))
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(raw)
        self.ctx = self.engine.create_execution_context()
        cud = _cudart()
        self.stream = ctypes.c_void_p()
        _ck(cud.cudaStreamCreate(ctypes.byref(self.stream)), "stream")
        self.bufs, self.host = {}, {}
        self.inputs, self.outputs = [], []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            arr = np.empty(shape, dtype)
            ptr = ctypes.c_void_p()
            _ck(cud.cudaMalloc(ctypes.byref(ptr), ctypes.c_size_t(arr.nbytes)), "malloc")
            self.bufs[name], self.host[name] = ptr, arr
            self.ctx.set_tensor_address(name, ptr.value)
            (self.inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else self.outputs).append(name)
        self.in_name = self.inputs[0]
        self.imgsz = self.host[self.in_name].shape[2]
        self.task = "segment" if len(self.outputs) >= 2 else "detect"
        self.infer_ms = 0.0
        self.gpu = GpuPost.create(self) if use_gpu_post else None
        log.info("TensorRT 추론기: %s, 앞뒤 처리 %s", path, "GPU" if self.gpu else "CPU(numpy)")

    def _letterbox(self, img):
        h, w = img.shape[:2]
        s = self.imgsz / max(h, w)
        nw, nh = int(round(w * s)), int(round(h * s))
        px, py = (self.imgsz - nw) // 2, (self.imgsz - nh) // 2
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, np.uint8)
        canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(self.host[self.in_name].dtype) / 255.0
        return np.ascontiguousarray(x), s, px, py

    def _run(self, x):
        cud = _cudart()
        H2D, D2H = 1, 2
        _ck(cud.cudaMemcpyAsync(self.bufs[self.in_name], x.ctypes.data_as(ctypes.c_void_p),
                                ctypes.c_size_t(x.nbytes), H2D, self.stream), "h2d")
        if not self.ctx.execute_async_v3(self.stream.value):
            raise RuntimeError("TensorRT 실행 실패")
        for n in self.outputs:
            a = self.host[n]
            _ck(cud.cudaMemcpyAsync(a.ctypes.data_as(ctypes.c_void_p), self.bufs[n], ctypes.c_size_t(a.nbytes),
                                    D2H, self.stream), "d2h")
        _ck(cud.cudaStreamSynchronize(self.stream), "sync")

    def infer(self, img, classes=None):
        t0 = time.monotonic()
        dets = None
        if self.gpu is not None and classes is not None and len(classes) == 1:
            try:
                dets = self._infer_gpu(img, int(classes[0]))
            except Exception as e:
                log.warning("GPU 앞뒤 처리 오류 → CPU 경로로 전환: %s", e)
                self.gpu = None
        if dets is None:
            dets = self._infer_host(img, classes)
        self.infer_ms = (time.monotonic() - t0) * 1000
        return dets

    def _poly(self, m, ps, s, px, py, w, h):
        """프로토 마스크(160) → 레터박스(640) → 원본 좌표 다각형."""
        cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return None
        c = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(np.float32)
        c = (c * ps - [px, py]) / s
        poly = np.clip(c, 0, [w - 1, h - 1]).astype(np.int32)
        return poly if len(poly) >= 3 else None

    def _nms(self, b, sc):
        xyxy = np.stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2, b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2], 1)
        idx = cv2.dnn.NMSBoxes([[float(v[0]), float(v[1]), float(v[2] - v[0]), float(v[3] - v[1])] for v in xyxy],
                               sc.tolist(), self.conf, self.iou)
        return xyxy, np.array(idx, int).reshape(-1)[:20]

    def _infer_gpu(self, img, cls_id):
        from .detector import Detection
        h, w = img.shape[:2]
        s, px, py = self.gpu.preprocess(img)
        if not self.ctx.execute_async_v3(self.stream.value):
            raise RuntimeError("TensorRT 실행 실패")
        cand = self.gpu.select(cls_id, self.conf)            # 여기서 GPU 완료를 기다린다(잠든 채)
        if not len(cand):
            return []
        sc = cand[:, 5]
        xyxy, idx = self._nms(cand[:, 1:5], sc)
        polys = [None] * len(idx)
        if self.gpu.nm and len(idx):
            ps = self.imgsz / self.gpu.ph
            ms = self.gpu.masks(cand[idx, 6:], xyxy[idx] / ps)
            polys = [self._poly(m, ps, s, px, py, w, h) for m in ms]
        dets = []
        for i, poly in zip(idx, polys):
            bx = np.clip((xyxy[i] - [px, py, px, py]) / s, 0, [w, h, w, h])
            dets.append(Detection(cls_id, self.names.get(cls_id, str(cls_id)), float(sc[i]),
                                  tuple(float(v) for v in bx), poly))
        return dets

    def _infer_host(self, img, classes=None):
        from .detector import Detection
        x, s, px, py = self._letterbox(img)
        self._run(x)
        outs = sorted((self.host[n] for n in self.outputs), key=lambda a: a.ndim)
        pred = outs[0][0].astype(np.float32)                   # (4+nc+nm, N)
        nm = 32 if self.task == "segment" else 0
        nc = pred.shape[0] - 4 - nm
        cls_scores = pred[4:4 + nc]
        if classes is not None:
            mask_c = np.zeros(nc, bool)
            mask_c[[c for c in classes if c < nc]] = True
            cls_scores = np.where(mask_c[:, None], cls_scores, 0)
        cls = cls_scores.argmax(0)
        score = cls_scores.max(0)
        keep = score > self.conf
        dets = []
        if keep.any():
            sc, cl = score[keep], cls[keep]
            xyxy, idx = self._nms(pred[:4, keep].T, sc)
            h, w = img.shape[:2]
            coef = pred[4 + nc:, keep].T if nm else None
            proto = outs[1][0].astype(np.float32) if nm else None
            for i in idx:
                bx = (xyxy[i] - [px, py, px, py]) / s
                bx = np.clip(bx, 0, [w, h, w, h])
                poly = None
                if nm:
                    m = 1 / (1 + np.exp(-(coef[i] @ proto.reshape(nm, -1)))).reshape(proto.shape[1:])
                    ps = self.imgsz / proto.shape[1]
                    x1, y1, x2, y2 = xyxy[i] / ps
                    crop = np.zeros_like(m)
                    crop[max(0, int(y1)):int(np.ceil(y2)), max(0, int(x1)):int(np.ceil(x2))] = 1
                    poly = self._poly((m * crop) > 0.5, ps, s, px, py, w, h)
                dets.append(Detection(int(cl[i]), self.names.get(int(cl[i]), str(cl[i])), float(sc[i]),
                                      tuple(float(v) for v in bx), poly))
        return dets
