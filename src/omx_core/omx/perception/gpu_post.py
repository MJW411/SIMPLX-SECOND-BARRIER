"""GPU 앞뒤 처리(omx/native/libomxgpu.so) ctypes 래퍼.

라이브러리가 없거나(빌드 전·nvcc 없음) 초기화에 실패하면 None 을 돌려주고, 추론기는 기존 numpy 경로를 쓴다.
빌드: tools/build_native.sh (run.sh 가 시작할 때 부른다).
"""
import ctypes
import logging
from pathlib import Path

import numpy as np

log = logging.getLogger("gpu_post")
LIB_PATH = Path(__file__).resolve().parent.parent / "native" / "libomxgpu.so"
_lib = None


def lib():
    global _lib
    if _lib is None:
        L = ctypes.CDLL(str(LIB_PATH))
        vp, i, f = ctypes.c_void_p, ctypes.c_int, ctypes.c_float
        L.og_blocking_sync.restype = i
        L.og_create.restype = vp
        L.og_create.argtypes = [i, i, i, i, i, i, i, i]
        L.og_destroy.argtypes = [vp]
        L.og_preprocess.restype = i
        L.og_preprocess.argtypes = [vp, vp, i, i, vp, vp, ctypes.POINTER(f), ctypes.POINTER(i), ctypes.POINTER(i)]
        L.og_select.restype = i
        L.og_select.argtypes = [vp, vp, ctypes.c_uint, f, vp, ctypes.POINTER(ctypes.POINTER(f))]
        L.og_masks.restype = i
        L.og_masks.argtypes = [vp, vp, vp, vp, i, vp, ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8))]
        _lib = L
    return _lib


def blocking_sync():
    """CUDA 를 처음 쓰기 전에 부른다: GPU 대기 중 CPU 가 바쁘게 돌지 않고 잠든다. 성공 여부를 돌려준다."""
    try:
        return lib().og_blocking_sync() == 0
    except OSError:
        return False


class GpuPost:
    MAX_MASKS = 20

    @classmethod
    def create(cls, yolo):
        if not LIB_PATH.exists():
            return None
        try:
            return cls(yolo)
        except Exception as e:                       # 무엇이 잘못돼도 파이썬 경로로 계속 동작
            log.warning("GPU 앞뒤 처리 사용 안 함: %s", e)
            return None

    def __init__(self, y):
        self.y = y
        outs = sorted(y.outputs, key=lambda n: y.host[n].ndim)
        self.pred_name = outs[0]
        self.proto_name = outs[1] if len(outs) > 1 else None
        dt = y.host[y.in_name].dtype
        if any(y.host[n].dtype != dt for n in y.outputs) or dt not in (np.float32, np.float16):
            raise ValueError(f"입출력 자료형이 섞임/미지원: {dt}")
        _, rows, self.N = y.host[self.pred_name].shape
        self.nm = y.host[self.proto_name].shape[1] if self.proto_name else 0
        self.nc = rows - 4 - self.nm
        ph, pw = y.host[self.proto_name].shape[2:] if self.proto_name else (1, 1)
        self.ph, self.pw = int(ph), int(pw)
        self.ctx = lib().og_create(y.imgsz, self.N, self.nc, self.nm, self.ph, self.pw, int(dt == np.float16), self.MAX_MASKS)
        if not self.ctx:
            raise RuntimeError("GPU 버퍼 할당 실패")

    def __del__(self):
        if getattr(self, "ctx", None) and _lib is not None:
            _lib.og_destroy(self.ctx)

    def preprocess(self, img):
        img = np.ascontiguousarray(img)
        h, w = img.shape[:2]
        s, px, py = ctypes.c_float(), ctypes.c_int(), ctypes.c_int()
        r = lib().og_preprocess(self.ctx, img.ctypes.data, w, h, self.y.bufs[self.y.in_name], self.y.stream,
                                ctypes.byref(s), ctypes.byref(px), ctypes.byref(py))
        if r != 0:
            raise RuntimeError(f"GPU 전처리 오류 {r}")
        return s.value, px.value, py.value

    def class_mask(self, classes):
        """클래스 번호 목록 → 비트마스크 (None = 전체). 32개 넘는 클래스는 GPU 경로를 쓸 수 없다."""
        if classes is None:
            classes = range(self.nc)
        m = 0
        for c in classes:
            c = int(c)
            if not 0 <= c < 32:
                raise ValueError(f"GPU 경로는 클래스 0~31만 지원: {c}")
            if c < self.nc:
                m |= 1 << c
        return m

    def select(self, cls_mask, conf):
        """점수 > conf 인 후보: (n, 7+nm) [anchor, cx, cy, w, h, score, cls, coef...] — 앵커 순서로 정렬.
        cls_mask 의 클래스들 중 최고 점수 클래스를 고른다 (numpy 경로와 같은 규칙)."""
        out = ctypes.POINTER(ctypes.c_float)()
        n = lib().og_select(self.ctx, self.y.bufs[self.pred_name], int(cls_mask), float(conf), self.y.stream, ctypes.byref(out))
        if n < 0:
            raise RuntimeError(f"GPU 후보 선택 오류 {-n}")
        if n == 0:
            return np.empty((0, 7 + self.nm), np.float32)
        a = np.ctypeslib.as_array(out, shape=(n, 7 + self.nm)).copy()
        return a[np.argsort(a[:, 0], kind="stable")]

    def masks(self, coef, box_proto):
        """coef (k, nm), box_proto (k, 4) 프로토 좌표 → (k, ph, pw) bool."""
        k = min(len(coef), self.MAX_MASKS)
        if k == 0:
            return np.zeros((0, self.ph, self.pw), bool)
        coef = np.ascontiguousarray(coef[:k], np.float32)
        box = np.ascontiguousarray(box_proto[:k], np.float32)
        out = ctypes.POINTER(ctypes.c_uint8)()
        n = lib().og_masks(self.ctx, self.y.bufs[self.proto_name], coef.ctypes.data, box.ctypes.data, k,
                           self.y.stream, ctypes.byref(out))
        if n < 0:
            raise RuntimeError(f"GPU 마스크 오류 {-n}")
        return np.ctypeslib.as_array(out, shape=(n, self.ph, self.pw)).astype(bool)
