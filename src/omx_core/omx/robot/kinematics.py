"""OMX-F 5축 기구학 (로봇 베이스 link0 기준, 단위 m / rad).

치수는 ROBOTIS 공식 URDF(omx_f_arm.urdf.xacro)에서 가져왔다.
  joint1 z축 (베이스 회전)   link0 → (-0.01125, 0, 0.034)
  joint2 y축 (어깨)          → (0, 0, 0.0635)
  joint3 y축 (팔꿈치)        → (0.0415, 0, 0.11315)
  joint4 y축 (손목 피치)     → (0.162, 0, 0)
  joint5 x축 (손목 롤)       → (0.0287, 0, 0)
  TCP (end_effector_link)    → (0.09193, -0.0016, 0)

TCP 자세는 (x, y, z, pitch)로 표현한다. pitch = joint2+joint3+joint4 로,
0이면 그리퍼가 수평 앞, +pi/2 면 아래를 향한다. 5축이라 좌우 방향(yaw)은 베이스 회전
atan2(y, x)에 묶이고, 손목 롤(joint5)은 위치에 영향이 거의 없어 따로 둔다.
"""
import math

import numpy as np

J1 = np.array([-0.01125, 0.0, 0.034])
J2_Z = 0.0635
L2X, L2Z = 0.0415, 0.11315
L2 = math.hypot(L2X, L2Z)
BETA = math.atan2(L2Z, L2X)          # 위팔의 0자세 각도 (수평 기준 위쪽)
L3 = 0.162
L4 = 0.0287 + 0.09193                # 손목 피치축 → TCP
TCP_Y = -0.0016
SHOULDER_Z = J1[2] + J2_Z            # 어깨축 높이


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def fk_points(q):
    """관절각 q(5) → 링크 관절점들 [base, j1, j2, j3, j4, j5, tcp] (7x3)."""
    q1, q2, q3, q4, q5 = q[:5]
    R = _rz(q1)
    p0 = np.zeros(3)
    p1 = J1.copy()
    p2 = p1 + R @ np.array([0, 0, J2_Z])
    R = R @ _ry(q2)
    p3 = p2 + R @ np.array([L2X, 0, L2Z])
    R = R @ _ry(q3)
    p4 = p3 + R @ np.array([L3, 0, 0])
    R = R @ _ry(q4)
    p5 = p4 + R @ np.array([0.0287, 0, 0])
    R = R @ _rx(q5)
    tcp = p5 + R @ np.array([0.09193, TCP_Y, 0])
    return np.array([p0, p1, p2, p3, p4, p5, tcp])


def fk(q):
    """관절각 → (x, y, z, pitch)."""
    p = fk_points(q)[-1]
    return np.array([p[0], p[1], p[2], q[1] + q[2] + q[3]])


class IKError(ValueError):
    pass


def ik(x, y, z, pitch, q5=0.0, q1_hint=None):
    """(x, y, z, pitch) → 관절각 5개. 팔꿈치 위(elbow-up) 해. 도달 불가면 IKError.

    손목 롤이 만드는 TCP의 y 오프셋(1.6 mm)은 무시한다.
    """
    dx, dy = x - J1[0], y - J1[1]
    r = math.hypot(dx, dy)
    if r < 1e-4:
        if q1_hint is None:
            raise IKError("베이스 축 위의 목표는 방향을 정할 수 없음")
        q1 = q1_hint
    else:
        q1 = math.atan2(dy, dx)
    # 어깨축 기준 평면 좌표 (수평 r, 위쪽 h)
    h = z - SHOULDER_Z
    wx = r - L4 * math.cos(pitch)
    wz = h + L4 * math.sin(pitch)
    d2 = wx * wx + wz * wz
    c = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if c > 1.0 or c < -1.0:
        raise IKError(f"도달할 수 없는 위치 (손목점 거리 {math.sqrt(d2):.3f} m)")
    gamma = -math.acos(c)                         # 음수 = 팔꿈치 위
    u2 = math.atan2(wz, wx) - math.atan2(L3 * math.sin(gamma), L2 + L3 * math.cos(gamma))
    u3 = u2 + gamma
    q2 = BETA - u2
    q3 = -u3 - q2
    q4 = pitch - q2 - q3
    return np.array([q1, q2, q3, q4, q5])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
