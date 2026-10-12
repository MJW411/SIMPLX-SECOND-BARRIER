"""마커 없는 자동 카메라-로봇 보정.

로봇(주황색 OMX)을 여러 자세로 옮기며 로봇 색 픽셀의 3D 점(카메라 기준)을 모으고,
FK로 계산한 링크 캡슐(선분 + 반경) 표면에 점들이 붙도록 강체 변환 T_base_cam 을 찾는다.
  1) 초기값: 자세별 (관측 점 중심 ↔ 모델 점 중심) 쌍으로 Kabsch
  2) 정밀화: 점-캡슐 표면 거리 최소화 (robust least squares, 6자유도)
카메라는 링크의 한쪽 면만 보므로 '축까지 거리 - 반경'을 오차로 써서 치우침을 없앤다.
"""
import math

import cv2
import numpy as np

from ..robot import kinematics as K
from ..safety.manager import STATE_KO
from .calib import kabsch

# 링크별 캡슐 반경 (m): [베이스 블록, 베이스기둥 j1-j2, 위팔 j2-j3, 아래팔 j3-j4, 손목 j4-j5, 그리퍼 j5-tcp]
RADII = np.array([0.030, 0.028, 0.022, 0.020, 0.020, 0.018])


def robot_points(frame, intr, cfg, max_pts=1500):
    """프레임에서 로봇 색 픽셀 → 3D 점 (카메라 기준)."""
    if frame.depth is None:
        raise ValueError("깊이 영상 없음 (영상 파일 소스로는 보정 불가)")
    rf = cfg["perception"].get("robot_filter", {})
    hsv = cv2.cvtColor(frame.color, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(rf.get("hsv_lo", [2, 190, 70]), np.uint8),
                    np.array(rf.get("hsv_hi", [13, 255, 255]), np.uint8))
    m = cv2.erode(m, np.ones((3, 3), np.uint8))            # 경계(깊이 튐) 제거
    vs, us = np.nonzero(m)
    z = frame.depth[vs, us].astype(np.float32) * frame.depth_scale
    ok = (z > 0.1) & (z < 2.0)
    vs, us, z = vs[ok], us[ok], z[ok]
    if len(z) > max_pts:
        idx = np.random.default_rng(0).choice(len(z), max_pts, replace=False)
        vs, us, z = vs[idx], us[idx], z[idx]
    return np.stack([(us - intr.cx) * z / intr.fx, (vs - intr.cy) * z / intr.fy, z], 1)


def model_samples(q, n=12):
    """FK 링크 축 위의 점들 (중심 계산용)."""
    p = K.fk_points(q)
    return np.concatenate([p[i] + (p[i + 1] - p[i]) * np.linspace(0, 1, n)[:, None] for i in range(len(p) - 1)])


def surface_residual(P, q):
    """점 P(N,3, 베이스 기준) → 가장 가까운 캡슐 표면까지 부호 없는 거리."""
    p = K.fk_points(q)
    A, B = p[:-1], p[1:]
    AB = B - A
    L2 = np.maximum((AB * AB).sum(1), 1e-9)
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(2) / L2[None], 0, 1)
    C = A[None] + t[..., None] * AB[None]
    d = np.linalg.norm(P[:, None, :] - C, axis=2) - RADII[None]
    return np.abs(d).min(1)


def _T(x):
    rv, t = x[:3], x[3:]
    R, _ = cv2.Rodrigues(rv.reshape(3, 1))
    return R, t


def solve(samples):
    """samples: [(q(6), P_cam(N,3))] → (T 4x4, 잔차 통계)."""
    from scipy.optimize import least_squares
    if len(samples) < 4:
        raise ValueError("자세 4개 이상 필요")
    # 1) 중심 쌍으로 초기값 (관측은 보이는 면만이라 치우치지만 초기값으로 충분)
    Pc = np.array([P.mean(0) for _, P in samples])
    Qc = np.array([model_samples(q).mean(0) for q, _ in samples])
    R0, t0 = kabsch(Pc, Qc)
    rv0, _ = cv2.Rodrigues(R0)
    x0 = np.concatenate([rv0.ravel(), t0])
    rng = np.random.default_rng(1)
    sub = [(q, P[rng.choice(len(P), min(len(P), 300), replace=False)]) for q, P in samples]

    def f(x):
        R, t = _T(x)
        return np.concatenate([surface_residual(P @ R.T + t, q) for q, P in sub])

    r = least_squares(f, x0, loss="huber", f_scale=0.01, max_nfev=200)
    R, t = _T(r.x)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    res = np.concatenate([surface_residual(P @ R.T + t, q) for q, P in samples])
    inl = res < 0.02
    return T, {"mean": float(np.median(res)), "max": float(np.percentile(res, 95)), "n": int(len(res)),
               "inlier": float(inl.mean()), "poses": len(samples)}


def calib_poses(yaw0, box, work_yaws=()):
    """보정용 자세 (TCP, 베이스 방향 yaw0 주변). 높이·거리·방향을 섞어 한 평면에 몰리지 않게.
    work_yaws: 실제 작업 방향(프로그램이 쓰는 자세들의 베이스 각도). 방향마다 3자세를 더해 사람이 다가오는
    작업 자리에서 가장 잘 맞게 한다 (2026-10-12: 8자세가 시작 방향 ±0.4 rad 에만 몰려 PICK·PLACE 쪽이 어긋났다).
    작업대에 닿지 않게 기존 자세와 같은 높이(10~20 cm)만 쓴다."""
    out = []
    for dyaw, r, z, pitch in [(0, 0.16, 0.16, 0.9), (0.35, 0.20, 0.10, 1.2), (-0.35, 0.20, 0.10, 1.2),
                              (0.2, 0.24, 0.20, 0.6), (-0.2, 0.24, 0.20, 0.6), (0, 0.12, 0.24, 0.4),
                              (0.4, 0.14, 0.14, 1.0), (-0.4, 0.14, 0.14, 1.0)]:
        y = yaw0 + dyaw
        out.append((-0.01125 + r * math.cos(y), r * math.sin(y), z, pitch))
    picked = []
    for wy in work_yaws:                                  # 시작 방향과 겹치거나 서로 가까운 방향은 하나로
        if abs(wy - yaw0) > 0.3 and all(abs(wy - p) > 0.3 for p in picked):
            picked.append(wy)
    for wy in sorted(picked[:2], key=lambda v: abs(v - yaw0)):   # 최대 2방향 × 3 = 6자세 (총 14)
        for dyaw, r, z, pitch in [(0, 0.18, 0.12, 1.1), (0.15, 0.22, 0.18, 0.7), (-0.15, 0.15, 0.20, 0.8)]:
            y = wy + dyaw
            out.append((-0.01125 + r * math.cos(y), r * math.sin(y), z, pitch))
    return out


def synthetic_test(seed=0):
    """합성 데이터로 풀이기 검증: 알려진 T 로 캡슐 표면의 카메라 쪽 점을 만들고 되찾는다."""
    rng = np.random.default_rng(seed)
    Rt, _ = cv2.Rodrigues(np.array([[0.2], [-1.4], [0.3]]))
    T_true = np.eye(4)
    T_true[:3, :3], T_true[:3, 3] = Rt, [0.05, -0.55, 0.25]
    Ti = np.linalg.inv(T_true)
    cam_in_base = T_true[:3, 3]
    samples = []
    for x, y, z, p in calib_poses(0.0, None):
        q = np.concatenate([K.ik(x, y, z, p), [0.3]])
        pts = K.fk_points(q)
        P = []
        for i in range(6):
            a, b = pts[i], pts[i + 1]
            for _ in range(80):
                c = a + (b - a) * rng.random()
                axis = (b - a) / (np.linalg.norm(b - a) + 1e-9)
                v = cam_in_base - c
                v -= axis * (v @ axis)                    # 카메라 쪽 방향 (축에 수직)
                v /= np.linalg.norm(v) + 1e-9
                ang = rng.uniform(-1.2, 1.2)              # 보이는 반원
                w = np.cross(axis, v)
                n = v * math.cos(ang) + w * math.sin(ang)
                P.append(c + n * RADII[i])
        P = np.array(P) + rng.normal(0, 0.003, (len(P), 3))
        Pc = P @ Ti[:3, :3].T + Ti[:3, 3]
        samples.append((q, Pc))
    T, st = solve(samples)
    err_t = np.linalg.norm(T[:3, 3] - T_true[:3, 3])
    err_r = math.degrees(np.arccos(np.clip((np.trace(T[:3, :3].T @ T_true[:3, :3]) - 1) / 2, -1, 1)))
    return err_t, err_r, st


class AutoCalibrator:
    """제어판 버튼 하나로: 보정 자세들을 차례로 돌며 로봇 점을 모으고 풀어서 저장한다."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.status = ""
        self.running = False

    def start(self, work_yaws=()):
        import threading
        self.work_yaws = [float(v) for v in (work_yaws or [])]
        if self.running:
            raise RuntimeError("이미 실행 중")
        ex = self.ctx.ex
        ex._require_ready()
        if self.ctx.safety.state not in ("NORMAL", "CONVEYOR"):
            raise RuntimeError(f"안전 상태가 정상일 때만 가능 (현재: {STATE_KO.get(self.ctx.safety.state, self.ctx.safety.state)})")
        self.running = True
        threading.Thread(target=self._run, daemon=True, name="autocalib").start()

    def _run(self):
        import time
        ctx, ex = self.ctx, self.ctx.ex
        q_start = ex.current_q()
        samples = []
        try:
            poses = calib_poses(q_start[0], None, getattr(self, "work_yaws", ()))
            for i, tgt in enumerate(poses):
                self.status = f"보정 자세 {i + 1}/{len(poses)}로 이동 중"
                try:
                    q = ex.solve(*tgt)
                except Exception as e:
                    self.status = f"자세 {i + 1} 건너뜀: {e}"
                    continue
                m = ex.move_joints(q, f"calib{i + 1}")
                if m.wait(90) != "done":
                    raise RuntimeError(f"이동 실패 ({ {'cancelled': '취소', 'failed': '실패'}.get(m.status, '시간 초과') }): {m.reason}")
                time.sleep(1.0)
                pts = []
                last = -1
                for _ in range(3):
                    while True:
                        f = ctx.cam.latest()
                        if f is not None and f.seq != last:
                            break
                        time.sleep(0.02)
                    last = f.seq
                    pts.append(robot_points(f, ctx.cam.intr, ctx.cfg, 600))
                P = np.concatenate(pts)
                if len(P) < 200:
                    self.status = f"자세 {i + 1}: 로봇 점 부족 ({len(P)}개) — 조명·색상 설정 확인"
                    continue
                samples.append((ex.state.q.copy(), P))
            self.status = "계산 중"
            T, st = solve(samples)
            cal = ctx.calib
            cal._backup()
            cal.T_base_cam = T
            cal.residual = {"mean": st["mean"], "max": st["max"], "n": st["n"], "method": "auto",
                            "inlier": st["inlier"], "poses": st["poses"]}
            cal.t = time.strftime("%Y-%m-%d %H:%M:%S")
            cal._save()
            ctx.per._fits = []                    # 새 보정으로 일치율을 다시 잰다
            ctx.per.calib_valid = True
            np.savez_compressed(ctx.cfg.path("autocalib_samples.npz"),
                                q=np.array([q for q, _ in samples]),
                                **{f"P{k}": P for k, (_, P) in enumerate(samples)})
            self.status = (f"완료 — 표면 잔차 중앙 {st['mean'] * 1000:.1f}mm, 95% {st['max'] * 1000:.1f}mm, "
                           f"정상점 {st['inlier'] * 100:.0f}%, 자세 {st['poses']}개")
            ctx.hub.emit("calib_auto", "자동 보정 " + self.status, level="info")
        except Exception as e:
            self.status = f"실패: {e}"
            ctx.hub.emit("calib_auto", f"자동 보정 실패: {e}", level="warn")
        finally:
            try:
                m = ex.move_joints(q_start, "calib_return")
                m.wait(90)
            except Exception:
                pass
            self.running = False
