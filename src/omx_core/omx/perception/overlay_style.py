"""작업 구역 영상 오버레이 그리기 (2026-10-12 사용자: 눈에 띄지만 투박하다 → 시안 B).

- 로봇: 계산한 로봇 모습을 아주 옅은 하늘색 면 + 가늘고 매끈한 테두리 + 은은한 빛 번짐, 관절은 작은 흰 점,
  그리퍼 끝(TCP)은 분홍 고리.
- 사람·헬멧: 마스크 면은 옅게, 테두리는 매끈하게(안티에일리어싱).
- 라벨: 어두운 반투명 바탕 상자 안에 글자. 서로 겹치면 위로 비켜 놓는다.
"""
import cv2
import numpy as np

ROBOT = (255, 215, 80)        # 하늘색 (BGR)
TCP = (22, 115, 249)          # 그리퍼 끝 고리: 주황 (BGR, #f97316) — 2026-10-12 사용자: 흰 선 + 주황 고리
SKEL = (255, 255, 255)        # 로봇 팔 선: 흰색
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _blend(img, mask, col, a):
    t = img.copy()
    t[mask] = col
    return cv2.addWeighted(t, a, img, 1 - a, 0)


def _contours(mask):
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return cs


def draw_robot(img, sil, joints_uv=None, tcp_uv=None):
    """sil: 로봇 모습 마스크(bool), joints_uv: 관절 픽셀들, tcp_uv: 그리퍼 끝 픽셀."""
    if sil is not None and sil.any():
        m = cv2.GaussianBlur(sil.astype(np.float32), (0, 0), 3) > 0.5          # 모서리를 둥글고 매끈하게
        cs = _contours(m)
        img = _blend(img, m, ROBOT, 0.14)
        g = img.copy()
        cv2.drawContours(g, cs, -1, ROBOT, 6, cv2.LINE_AA)
        img = cv2.addWeighted(cv2.GaussianBlur(g, (0, 0), 2), 0.35, img, 0.65, 0)   # 은은한 빛 번짐
        cv2.drawContours(img, cs, -1, ROBOT, 1, cv2.LINE_AA)
    for p in ([] if joints_uv is None else np.asarray(joints_uv).astype(int)):
        cv2.circle(img, (int(p[0]), int(p[1])), 4, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (int(p[0]), int(p[1])), 4, ROBOT, 1, cv2.LINE_AA)
    if tcp_uv is not None:
        p = (int(tcp_uv[0]), int(tcp_uv[1]))
        cv2.circle(img, p, 8, TCP, 1, cv2.LINE_AA)
        cv2.circle(img, p, 2, TCP, -1, cv2.LINE_AA)
    return img


def draw_skeleton(img, uv):
    """로봇 팔 선 (2026-10-12 사용자: 모양 전체 칠하기는 별로 → 원래 선으로, 색만 연하게).
    uv: 베이스부터 그리퍼 끝까지 관절 픽셀 (fk_points 투영). 가는 연하늘색 선을 반투명으로, 그리퍼 끝은 작은 분홍 고리."""
    if uv is None or len(uv) < 2:
        return img
    p = np.asarray(uv, float).round().astype(int)
    t = img.copy()
    for a, b in zip(p[1:-1], p[2:]):
        cv2.line(t, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), SKEL, 2, cv2.LINE_AA)
    img = cv2.addWeighted(t, 0.6, img, 0.4, 0)
    e = (int(p[-1][0]), int(p[-1][1]))
    cv2.circle(img, e, 6, TCP, 1, cv2.LINE_AA)
    cv2.circle(img, e, 2, TCP, -1, cv2.LINE_AA)
    return img


def mask_outline(img, poly, col):
    """사람·헬멧 마스크 테두리 (매끈하게). 면은 호출한 쪽이 layer 에 칠해 한 번에 섞는다."""
    cv2.polylines(img, [poly], True, col, 1, cv2.LINE_AA)


class Labels:
    """라벨 상자를 겹치지 않게 놓는다 (프레임마다 새로 만든다)."""

    def __init__(self, img):
        self.h, self.w = img.shape[:2]
        self.placed = []

    def _free(self, r):
        return all(r[2] <= q[0] or r[0] >= q[2] or r[3] <= q[1] or r[1] >= q[3] for q in self.placed)

    def put(self, img, text, x, y, col):
        """(x, y) = 라벨 상자 왼쪽 아래가 놓일 자리 (보통 마스크 맨 위). 겹치면 위로, 화면 위에 닿으면 아래로."""
        (tw, th), base = cv2.getTextSize(text, FONT, 0.45, 1)
        bw, bh = tw + 10, th + base + 6
        x = int(min(max(0, x), self.w - bw))
        y0 = int(y) - 3
        r = None
        for k in range(8):
            for dy in (-k * (bh + 2), k * (bh + 2)):
                top = min(max(0, y0 - bh + dy), self.h - bh)
                cand = (x, top, x + bw, top + bh)
                if self._free(cand):
                    r = cand
                    break
            if r is not None:
                break
        r = r or (x, max(0, y0 - bh), x + bw, max(0, y0 - bh) + bh)
        self.placed.append(r)
        sub = img[r[1]:r[3], r[0]:r[2]]
        img[r[1]:r[3], r[0]:r[2]] = cv2.addWeighted(sub, 0.35, np.full_like(sub, 20), 0.65, 0)   # 어두운 반투명 바탕
        cv2.rectangle(img, (r[0], r[1]), (r[2] - 1, r[3] - 1), col, 1, cv2.LINE_AA)
        cv2.putText(img, text, (r[0] + 5, r[3] - base - 3), FONT, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return img
