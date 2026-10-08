"""사람 한 명의 3D 점·거리 계산 (인식 루프에서 사람마다 프레임당 한 번).

C++ 구현(omx/native/libomxfast.so, omxfast.cpp)이 있으면 그것을 쓰고, 없으면 같은 계산을 하는 numpy 구현을 쓴다.
두 구현은 결과가 같아야 한다 (tests/test_core.py 가 무작위 사례로 비교).

결과 dict: npts, cam_dist, dist, on_frac, near_frac, pos(3), n_off  — 깊이 픽셀이 부족하면 None.
 - dist: 보정·링크가 있으면 로봇 링크 표면까지 최소거리(몸체 점 제외, 모두 몸체 위면 0), 보정 없으면 카메라까지.
 - pos : 대표 위치(축별 중앙값). 보정 시 몸체 밖 점 기준, 몸체 밖 점이 없으면 전체 점 기준.
"""
import ctypes
from pathlib import Path

import cv2
import numpy as np

LIB_PATH = Path(__file__).resolve().parent.parent / "native" / "libomxfast.so"
NEAR_BASE_R = 0.030          # 베이스 블록 반경 (근접 비율 계산용)
_lib = None


def _load():
    global _lib
    if _lib is None:
        try:
            L = ctypes.CDLL(str(LIB_PATH))
        except OSError:
            _lib = False
            return _lib
        d, i, f, vp = ctypes.c_double, ctypes.c_int, ctypes.c_float, ctypes.c_void_p
        L.of_person.restype = i
        L.of_person.argtypes = [vp, vp, i, i, i, i, i, f, f, f, i, i, d, d, d, d, vp, vp, i, vp, d, d, vp]
        _lib = L
    return _lib


def native_available():
    return bool(_load())


def seg_dist(P, A, B):
    """점들 P(N,3)와 선분들 A,B(M,3) 사이 거리 행렬 (N,M)."""
    AB = B - A
    L2 = np.maximum((AB * AB).sum(1), 1e-9)
    AP = P[:, None, :] - A[None]
    t = np.clip((AP * AB[None]).sum(2) / L2[None], 0, 1)
    C = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(P[:, None, :] - C, axis=2)


def person_geometry(mask, depth, depth_scale, intr, T, links, radii, pc, margin, near, use_native=True):
    L = _load() if use_native else False
    if not L:
        return person_geometry_py(mask, depth, depth_scale, intr, T, links, radii, pc, margin, near)
    m = np.ascontiguousarray(mask.view(np.uint8) if mask.dtype == bool else mask, np.uint8)
    dep = np.ascontiguousarray(depth, np.uint16)
    if m.shape != dep.shape:
        raise ValueError("마스크와 깊이 크기가 다름")
    x0, y0, w, h = cv2.boundingRect(m)                 # 마스크를 감싸는 사각형만 훑는다
    if w == 0 or h == 0:
        return None
    stride = dep.shape[1]
    Tc = np.ascontiguousarray(T, np.float64) if T is not None else None
    Lc = np.ascontiguousarray(links, np.float64) if (T is not None and links is not None) else None
    rc = np.ascontiguousarray(radii, np.float64)
    out = np.zeros(9, np.float64)
    lo, hi = pc["depth_range"]
    r = L.of_person(m.ctypes.data + y0 * stride + x0, dep.ctypes.data + (y0 * stride + x0) * 2, w, h, stride, x0, y0,
                    float(depth_scale), float(lo), float(hi),
                    int(pc["min_depth_pixels"]), int(pc["max_points"]), intr.fx, intr.fy, intr.cx, intr.cy,
                    Tc.ctypes.data if Tc is not None else None, Lc.ctypes.data if Lc is not None else None,
                    len(Lc) if Lc is not None else 0, rc.ctypes.data, float(margin), float(near), out.ctypes.data)
    if r != 0:
        return None
    return {"npts": int(out[8]) if Lc is not None else int(out[0]), "cam_dist": float(out[1]), "dist": float(out[2]),
            "on_frac": float(out[3]), "near_frac": float(out[4]), "pos": out[5:8].copy(), "n_off": int(out[8])}


def person_geometry_py(mask, depth, depth_scale, intr, T, links, radii, pc, margin, near):
    """numpy 기준 구현 (C++ 과 같은 순서·같은 식)."""
    vs, us = np.nonzero(mask)
    z = depth[vs, us].astype(np.float32) * np.float32(depth_scale)
    lo, hi = pc["depth_range"]
    ok = (z > lo) & (z < hi)
    vs, us, z = vs[ok], us[ok], z[ok]
    if len(z) < pc["min_depth_pixels"]:
        return None
    med = np.median(z)
    keep = np.abs(z - med) < max(np.float32(0.08), np.float32(0.25) * med)     # 배경 섞임 제거
    vs, us, z = vs[keep], us[keep], z[keep]
    k, mp = len(z), pc["max_points"]
    if k > mp:                                        # 고른 간격으로 추림 (재현 가능, C++ 과 같음)
        idx = (np.arange(mp, dtype=np.int64) * k) // mp
        vs, us, z = vs[idx], us[idx], z[idx]
    z = z.astype(np.float64)
    P = np.stack([(us - intr.cx) * z / intr.fx, (vs - intr.cy) * z / intr.fy, z], 1)
    n = len(P)
    cam_dist = float(np.linalg.norm(P, axis=1).min())
    res = {"npts": n, "cam_dist": cam_dist, "dist": cam_dist, "on_frac": 0.0, "near_frac": 0.0, "n_off": n}
    if T is not None:
        P = P @ T[:3, :3].T + T[:3, 3]
        res["dist"] = float("inf")
        if links is not None:
            D = seg_dist(P, links[1:-1], links[2:]) - radii[None]
            dmin = D.min(1)
            off = ~(dmin < margin)
            Dn = seg_dist(P, links[:-1], links[1:]) - np.r_[NEAR_BASE_R, radii][None]
            res["near_frac"] = float((Dn.min(1) < near).mean())
            res["on_frac"] = float((~off).mean())
            res["n_off"] = res["npts"] = int(off.sum())
            res["dist"] = float(max(0.0, dmin[off].min())) if off.any() else 0.0
            if off.any():
                P = P[off]
    res["pos"] = np.median(P, 0)
    return res
