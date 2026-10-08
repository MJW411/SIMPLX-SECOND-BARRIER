"""카메라-로봇 보정: T_base_cam (카메라 좌표 → 로봇 베이스 좌표) 4x4.

방법: 로봇을 여러 자세로 옮기며 (카메라에서 본 그리퍼 끝 3D 점, FK로 계산한 TCP) 쌍을 모아
Kabsch(최소제곱 강체 정합)로 푼다. 4쌍 이상, 한 평면에 있지 않게.
그리퍼 끝 점은 두 가지로 얻는다.
  1) ArUco 마커(DICT_4X4_50)를 그리퍼 끝에 붙이고 자동 검출 (마커 중심 = TCP 로 가정,
     오프셋이 있으면 marker_offset 설정)
  2) 제어판 영상에서 그리퍼 끝을 직접 클릭 (마커 인쇄 없이 바로 가능)
깊이는 클릭/마커 중심 주변 5x5의 유효 깊이 중앙값.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np


def kabsch(P, Q):
    """R,t 를 구해 R@P + t ≈ Q. P,Q: (N,3)."""
    cp, cq = P.mean(0), Q.mean(0)
    H = (P - cp).T @ (Q - cq)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = cq - R @ cp
    return R, t


class Calibration:
    def __init__(self, path):
        self.path = Path(path)
        self.T_base_cam = None
        self.samples = []            # [{cam:[3], base:[3], px:[2]}]
        self.residual = None
        self.t = None
        if self.path.exists():
            d = json.loads(self.path.read_text())
            self.T_base_cam = np.array(d["T_base_cam"]) if d.get("T_base_cam") else None
            self.residual = d.get("residual")
            self.t = d.get("t")
            self.samples = d.get("samples", [])

    def _backup(self):
        """보정 결과가 바뀌기 전 이전 결과를 data/calib_history/ 에 남긴다 (실수로 지워도 되돌릴 수 있게)."""
        if self.path.exists() and self.T_base_cam is not None:
            import shutil
            d = self.path.parent / "calib_history"
            d.mkdir(exist_ok=True)
            shutil.copy2(self.path, d / f"calib_{time.strftime('%Y%m%d_%H%M%S')}.json")

    def _save(self):
        self.path.write_text(json.dumps({
            "T_base_cam": None if self.T_base_cam is None else self.T_base_cam.tolist(),
            "residual": self.residual, "t": self.t, "samples": self.samples}, indent=1))

    @staticmethod
    def pixel_to_cam(frame, intr, u, v, wins=(2, 5, 9, 14)):
        """클릭 지점의 3D 점. 그리퍼 끝처럼 가는 부분은 깊이 구멍이 흔해서 주변을 넓혀 가며 찾고,
        뒤 배경이 섞이지 않게 가장 앞쪽(가까운) 깊이 무리만 쓴다."""
        if frame.depth is None:
            raise ValueError("깊이 데이터 없음")
        u, v = int(round(u)), int(round(v))
        for win in wins:
            patch = frame.depth[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1].astype(float) * frame.depth_scale
            z = patch[(patch > 0.08) & (patch < 3.0)]
            if len(z) >= 3:
                break
        else:
            raise ValueError(f"클릭한 곳 주변 {wins[-1]}px 안에 깊이값 없음 — 그리퍼 끝 바로 옆(몸체 쪽)을 클릭")
        near = np.percentile(z, 10)
        z = float(np.median(z[z < near + 0.02]))
        return np.array([(u - intr.cx) * z / intr.fx, (v - intr.cy) * z / intr.fy, z])

    @staticmethod
    def find_aruco(img):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None or len(ids) == 0:
            return None
        c = corners[0][0].mean(0)
        return float(c[0]), float(c[1])

    def add_sample(self, frame, intr, tcp_base, u=None, v=None):
        if u is None:
            uv = self.find_aruco(frame.color)
            if uv is None:
                raise ValueError("ArUco 마커 미검출 (DICT_4X4_50)")
            u, v = uv
        p = self.pixel_to_cam(frame, intr, u, v)
        self.samples.append({"cam": p.tolist(), "base": [float(x) for x in tcp_base[:3]], "px": [u, v]})
        self._save()
        return len(self.samples)

    def solve(self):
        if len(self.samples) < 4:
            raise ValueError("점 4개 이상 필요")
        P = np.array([s["cam"] for s in self.samples])
        Q = np.array([s["base"] for s in self.samples])
        if np.linalg.matrix_rank(Q - Q.mean(0), tol=0.01) < 3:
            raise ValueError("로봇 자세가 한 평면·직선에 몰려 있음 — 높이와 방향을 바꿔 더 수집 필요")
        self._backup()
        R, t = kabsch(P, Q)
        err = np.linalg.norm(P @ R.T + t - Q, axis=1)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        self.T_base_cam = T
        self.residual = {"mean": float(err.mean()), "max": float(err.max()), "n": len(err)}
        self.t = time.strftime("%Y-%m-%d %H:%M:%S")
        self._save()
        return self.residual

    def reset(self, keep_T=True):
        """수동 보정 점만 지운다 (보정 결과는 유지). 결과까지 지우려면 delete_result()."""
        self.samples = []
        if not keep_T:
            self._backup()
            self.T_base_cam = None
            self.residual = None
        self._save()

    def delete_result(self):
        """보정 결과 삭제 (이전 결과는 calib_history 에 백업)."""
        self.reset(keep_T=False)

    def project(self, pts_base, intr):
        """베이스 좌표 점들 → 이미지 픽셀 (보정 시)."""
        if self.T_base_cam is None or intr is None:
            return None
        Ti = np.linalg.inv(self.T_base_cam)
        P = pts_base @ Ti[:3, :3].T + Ti[:3, 3]
        z = np.maximum(P[:, 2], 1e-3)
        return np.stack([P[:, 0] * intr.fx / z + intr.cx, P[:, 1] * intr.fy / z + intr.cy], 1)

    def snapshot(self):
        return {"calibrated": self.T_base_cam is not None, "samples": len(self.samples),
                "residual": self.residual, "t": self.t}
