"""사람 인식 스레드: 검출 → 마스크 깊이 → 3D 점 → 로봇 링크 최소거리.

- 마스크 안 깊이만 쓰고, 중앙값에서 크게 벗어난 점(배경)은 버린다.
- 카메라-로봇 보정(T_base_cam)이 있으면 로봇 베이스 기준으로 바꿔 FK 링크(캡슐)와의
  최소거리를 구한다. 보정 전에는 카메라에서 사람까지 거리로 임시 판정한다.
- 깊이를 믿을 수 없는 사람(유효 픽셀 부족)은 거리 0(가장 가까움)으로 본다 — 모르면 안전 쪽.
"""
import logging
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .detector import det_mask
from .geom import person_geometry, seg_dist  # noqa: F401  (seg_dist: 기존 호출·시험 호환)

log = logging.getLogger("people")
LINK_RADIUS = 0.035          # 링크 두께 반경 (m) — 보정 전 호환용
# 링크별 실제 반경 (m): [j1-j2 기둥, 위팔, 아래팔, 손목, 그리퍼]. 거리 = 링크 축까지 거리 - 반경
LINK_RADII = np.array([0.028, 0.022, 0.020, 0.020, 0.018])
ON_ROBOT_MARGIN = 0.015      # 링크 표면에서 이 안의 점은 로봇 몸체(깊이 노이즈 포함)로 보고 뺀다
# 로봇 자기 오검출 판정 (둘 다 만족할 때만 버린다):
#  - 검출 마스크의 ROBOT_SELF_OVERLAP 이상이 영상 속 로봇 모습(FK 투영) 위에 있고
#  - 점의 ROBOT_SELF_FRAC 이상이 로봇 표면에서 ROBOT_SELF_NEAR 안에 있다 (검은 케이블·브래킷·그리퍼 포함)
# 실측(2026-09-27): YOLO가 로봇을 person 0.60~0.63으로 잡고, 그 마스크 점의 대부분은 주황 몸체가 아닌
# 케이블·브래킷·그리퍼로 모델 표면에서 1.5~4 cm 떨어져 있었다. 주황 몸체 점은 98%가 1.5 cm 안.
ROBOT_SELF_OVERLAP = 0.5
ROBOT_SELF_FRAC = 0.7
ROBOT_SELF_NEAR = 0.04


def nnz(b):
    """bool 마스크의 참 개수 (numpy .sum() 보다 수 배 빠르다)."""
    return cv2.countNonZero(b.view(np.uint8))


@dataclass
class Person:
    box: tuple
    conf: float
    px: tuple                    # 이미지 중심 (u, v)
    pos: np.ndarray = None       # 대표 3D 위치 (보정 시 베이스 기준, 아니면 카메라 기준)
    dist: float = 0.0            # 로봇(또는 카메라)까지 최소거리
    cam_dist: float = 0.0        # 카메라에서 가장 가까운 점까지 거리
    npts: int = 0
    in_zone: bool = True
    poly: np.ndarray = None
    overlap: float = 0.0         # 로봇 모습과 겹친 비율 (진단용)
    near: float = 0.0            # 로봇 표면 4 cm 안 점 비율 (진단용)


@dataclass
class Percept:
    t: float
    seq: int
    persons: list = field(default_factory=list)
    min_dist: float = float("inf")
    calibrated: bool = False
    infer_ms: float = 0.0
    shape: tuple = (480, 640)
    brightness: float = 255.0    # 영상 평균 밝기 (조도 부족이면 인식을 믿을 수 없다)


def link_distance(P, links):
    """사람 점 P(N,3, 베이스 기준)와 FK 링크 점 links(7,3) → (최소거리, 로봇 몸체 위 점 비율, 몸체 밖 점 마스크).
    링크 표면(반경+여유) 안의 점은 로봇 몸체로 보고 거리 계산에서 뺀다."""
    D = seg_dist(P, links[1:-1], links[2:]) - LINK_RADII[None]
    dmin = D.min(1)
    on_robot = dmin < ON_ROBOT_MARGIN
    off = ~on_robot
    dist = float(max(0.0, dmin[off].min())) if off.any() else 0.0
    return dist, float(on_robot.mean()) if len(P) else 0.0, off


def link_distance_near_frac(P, links, near):
    """점 중 로봇 표면(베이스 블록 포함)에서 near 안에 있는 비율."""
    if not len(P):
        return 0.0
    D = seg_dist(P, links[:-1], links[1:]) - np.r_[0.030, LINK_RADII][None]
    return float((D.min(1) < near).mean())


class PeoplePerception:
    def __init__(self, camera, detector, cfg, link_points_fn, calib):
        self.cam = camera
        self.det = detector
        self.cfg = cfg
        self.link_points = link_points_fn
        self.calib = calib
        self.latest = None
        self.listeners = []
        self.calib_valid = calib.T_base_cam is not None if calib is not None else False
        self.calib_fit = None
        self.fps = 0.0
        self._run = True
        self.thread = threading.Thread(target=self._loop, name="people", daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        last = -1
        n, t0 = 0, time.monotonic()
        while self._run:
            f = self.cam.latest()
            if f is None or f.seq == last:
                time.sleep(0.005)
                continue
            last = f.seq
            try:
                p = self.process(f)
            except Exception:
                log.exception("perception")
                time.sleep(0.2)
                continue
            self.latest = (f, p)
            for cb in self.listeners:
                cb(f, p)
            n += 1
            if time.monotonic() - t0 >= 2.0:
                self.fps = n / (time.monotonic() - t0)
                n, t0 = 0, time.monotonic()

    def robot_mask(self, img):
        """로봇(주황색 OMX) 픽셀 마스크. 로봇 팔이 사람으로 오검출되는 것을 막는다.
        주의: 주황색 옷을 입은 사람도 걸러질 수 있다 — robot_filter.enabled 로 끌 수 있음."""
        rf = self.cfg["perception"].get("robot_filter", {})
        if not rf.get("enabled", True):
            return None
        if self.calib is not None and self.calib.T_base_cam is not None and self.calib_valid:
            return None     # 보정이 맞을 때: 2D 마스크 대신 3D로 로봇 몸체 점만 뺀다 (process 참고)
        return self.color_mask(img)

    def color_mask(self, img):
        rf = self.cfg["perception"].get("robot_filter", {})
        small = cv2.resize(img, (img.shape[1] // 4, img.shape[0] // 4), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        lo, hi = rf.get("hsv_lo", [2, 190, 70]), rf.get("hsv_hi", [13, 255, 255])
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cv2.dilate(m, np.ones((3, 3), np.uint8))
        return cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0

    def check_calibration(self, f):
        """보정이 지금 영상과 맞는지 확인: 주황색 로봇 픽셀 중 FK로 투영한 로봇 모습 안에 드는 비율.
        카메라가 옮겨지면 비율이 떨어진다 → 보정을 쓰지 않고(카메라 기준 거리·색 필터) 다시 보정하라고 알린다."""
        T = self.calib.T_base_cam
        if T is None or self.cam.intr is None:
            self.calib_valid, self.calib_fit = False, None
            return
        links = self.link_points()
        if links is None:
            return
        orange = self.color_mask(f.color)
        n = nnz(orange)
        if n < 1500:                      # 로봇이 잘 안 보임(어두움·가림) → 판단 보류
            return
        sil = self._fk_silhouette(f.color.shape, T, links)
        fit = nnz(orange & sil) / n
        self._fits = (getattr(self, "_fits", []) + [fit])[-5:]
        self.calib_fit = sum(self._fits) / len(self._fits)
        if self.calib_fit < 0.5 and len(self._fits) >= 3:
            self.calib_valid = False
        elif self.calib_fit > 0.65:
            self.calib_valid = True

    def _fk_silhouette(self, shape, T, links):
        """영상 속 로봇 모습: FK 링크를 실제 폭(반경+1 cm, 그리퍼는 더 넓게)으로 투영한 마스크."""
        intr = self.cam.intr
        Ti = np.linalg.inv(T)
        P = links @ Ti[:3, :3].T + Ti[:3, 3]
        m = np.zeros(shape[:2], np.uint8)
        radii = [0.035] + list(LINK_RADII[:-1] + 0.01) + [0.035]      # 베이스, 링크들, 그리퍼
        for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
            if a[2] < 0.05 or b[2] < 0.05:
                continue
            ua = (int(a[0] * intr.fx / a[2] + intr.cx), int(a[1] * intr.fy / a[2] + intr.cy))
            ub = (int(b[0] * intr.fx / b[2] + intr.cx), int(b[1] * intr.fy / b[2] + intr.cy))
            th = int(2 * radii[min(k, len(radii) - 1)] * intr.fx / max(0.1, min(a[2], b[2])))
            cv2.line(m, ua, ub, 1, max(3, th))
        return m > 0

    def process(self, f):
        pc = self.cfg["perception"]
        now = time.monotonic()
        if now - getattr(self, "_calib_check_t", 0) > 1.0:
            self._calib_check_t = now
            self.check_calibration(f)
        dets = self.det.infer(f.color, classes=[pc["person_class"]])
        rmask = self.robot_mask(f.color) if dets else None
        self.robot_rejects = getattr(self, "robot_rejects", 0)
        intr = self.cam.intr
        T = self.calib.T_base_cam if self.calib_valid else None     # 보정이 틀어졌으면 쓰지 않는다
        links = self.link_points() if T is not None else None
        sil = self._fk_silhouette(f.color.shape, T, links) if (dets and T is not None and links is not None
                                                                and self.cam.intr is not None) else None
        out = Percept(f.t, f.seq, calibrated=T is not None, infer_ms=self.det.infer_ms, shape=f.color.shape[:2])
        out.brightness = float(f.color[::8, ::8].mean())
        zone = self.cfg["safety"]["zone_base"]
        frac_max = pc.get("robot_filter", {}).get("max_frac", 0.25)
        for d in dets:
            x1, y1, x2, y2 = d.box
            per = Person(d.box, d.conf, ((x1 + x2) / 2, (y1 + y2) / 2), poly=d.poly)
            m = det_mask(d, f.color.shape)
            area = max(1, nnz(m))
            overlap = nnz(m & sil) / area if sil is not None else 0.0
            if rmask is not None:
                if nnz(m & rmask) / area > frac_max:
                    self.robot_rejects += 1          # 로봇 자신을 사람으로 본 것 → 버림
                    continue
                m &= ~rmask                           # 사람 마스크에 섞인 로봇 픽셀은 거리 계산에서 뺀다
            if f.depth is not None and intr is not None:
                # 3D 판정(C++ 가속, 없으면 같은 numpy 계산): 링크 표면(반경+여유) 안의 점 = 로봇 몸체 → 뺀다. 나머지로 거리.
                # (2D 띠로 자르면 로봇 가까이 온 손이 통째로 지워져 근접 정지가 안 걸렸다 — 2026-09-27 실기)
                g = person_geometry(m, f.depth, f.depth_scale, intr, T, links, LINK_RADII, pc, ON_ROBOT_MARGIN, ROBOT_SELF_NEAR)
                if g is None:
                    per.dist = per.cam_dist = 0.0   # 깊이 불명 → 가장 가깝다고 본다
                else:
                    per.npts, per.cam_dist = g["npts"], g["cam_dist"]
                    if T is not None and links is not None:
                        per.overlap, per.near = overlap, g["near_frac"]
                        if overlap >= ROBOT_SELF_OVERLAP and g["near_frac"] >= ROBOT_SELF_FRAC:
                            self.robot_rejects += 1      # 로봇 모습 위 + 점 대부분이 로봇 표면 근처 → 로봇 자기 오검출
                            continue
                        per.dist = g["dist"]
                    elif T is None:
                        per.dist = g["dist"]            # 보정 전: 카메라까지 거리
                    per.pos = g["pos"]
                    if T is not None:
                        c = per.pos
                        per.in_zone = bool(zone["x"][0] <= c[0] <= zone["x"][1] and zone["y"][0] <= c[1] <= zone["y"][1]
                                           and zone["z"][0] <= c[2] <= zone["z"][1])
            else:
                per.dist = per.cam_dist = float("inf")   # 깊이 없는 영상 소스: 거리 판정 안 함
            out.persons.append(per)
        zone_d = [p.dist for p in out.persons if p.in_zone]
        out.min_dist = min(zone_d) if zone_d else float("inf")
        return out
