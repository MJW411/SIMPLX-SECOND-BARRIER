"""사라짐 판정 — 사람별 ID 없이 직전/현재 프레임 검출의 1:1 위치 매칭.

- 직전 프레임의 사람이 현재 프레임에서 매칭되지 않으면 '대기(pending)'로 둔다.
  * 출구(화면 가장자리, 출입문 영역, 감시 구역 밖)에서 사라졌으면 정상 퇴장 → 버린다.
  * 유예시간 안에 같은 위치 근처에서 다시 보이면 대기를 취소한다(짧은 인식 끊김).
  * 유예시간이 지나면 '미해결 사라짐'이 되고 관리자 확인으로만 해제된다.
- 대기 중인 사람의 마지막 거리는 유예시간 동안 계속 판단에 반영한다(원래 판단 유지).
- 다른 사람이 나타나도(다른 위치) 대기·미해결은 풀리지 않는다.
- 연속 vanish_min_frames 프레임 이상 보인(확립된) 검출만 사라짐 대상이 된다.
  (실기 시험: 걸린 옷이 1~2프레임 사람으로 깜박이는 오검출이 사라짐 정지를 일으킨 문제 대응)
- 출구: 화면 네 가장자리(박스가 닿음), 출입문 영역, 감시 구역 밖. (2026-10-08 실기: 위·아래로 나가는 손이 사라짐이 됐다)
- 들려 나감: 사라진 사람 자리에 겹쳐 있던 다른 검출(피규어를 집은 손)이 이어서 출구로 나가면 함께 나간 것으로 보고
  대기 중인 사라짐을 지운다. 겹친 검출이 화면 안에 남거나 출구로 나가지 않으면 그대로 사라짐 판정.
"""
import time

import numpy as np


def box_iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def overlap_small(a, b):
    """두 박스 겹친 넓이 / 작은 박스 넓이 (손이 피규어를 덮었는지)."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    small = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return ix * iy / small if small > 0 else 0.0


class VanishTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prev = []            # [(pos3 or None, px, dist)]
        self._prev_hits = []      # 직전 검출별 연속 관측 프레임 수
        self.pending = []         # [{pos, px, dist, t}]
        self.unresolved = []      # [{pos, px, t, dist}]
        self._prev_zone = []
        self._prev_box = []
        self._prev_carry = []     # 직전 검출별로 '함께 들고 있을 수 있는' 대기 사라짐 id
        self._pid = 0

    def _near(self, a_pos, a_px, b_pos, b_px, a_box=None, b_box=None):
        s = self.cfg["safety"]
        # 박스가 많이 겹치면 같은 사람 (실제 사람이 넘어질 때 몸 중심이 매칭 반경보다 크게 움직여도 놓치지 않게)
        if a_box is not None and b_box is not None and box_iou(a_box, b_box) >= s.get("vanish_match_iou", 0.3):
            return True
        if a_pos is not None and b_pos is not None:
            return float(np.linalg.norm(a_pos - b_pos)) < s["vanish_match_radius"]
        return float(np.hypot(a_px[0] - b_px[0], a_px[1] - b_px[1])) < 60

    def _is_exit(self, px, in_zone, shape):
        s = self.cfg["safety"]
        b = s["exit_border_px"]
        h, w = shape
        if px[0] < b or px[0] > w - b or px[1] < b or px[1] > h - b:
            return True
        roi = s.get("door_roi_px")
        if roi and roi[0] <= px[0] <= roi[2] and roi[1] <= px[1] <= roi[3]:
            return True
        return not in_zone

    def update(self, percept, now=None):
        """새 인식 결과 반영. 새로 확정된 사라짐 목록을 돌려준다."""
        now = time.monotonic() if now is None else now
        cur = [(p.pos, p.px, p.dist, p.in_zone, p.box) for p in percept.persons]
        # 직전 ↔ 현재 1:1 탐욕 매칭 (가까운 쌍부터)
        pairs = []
        for i, (ppos, ppx, _) in enumerate(self.prev):
            pbox = self._prev_box[i] if i < len(self._prev_box) else None
            for j, c in enumerate(cur):
                if self._near(ppos, ppx, c[0], c[1], pbox, c[4]):
                    d = (np.linalg.norm(ppos - c[0]) if ppos is not None and c[0] is not None
                         else np.hypot(ppx[0] - c[1][0], ppx[1] - c[1][1]) / 1000)
                    pairs.append((d, i, j))
        used_p, used_c, match = set(), set(), {}
        for d, i, j in sorted(pairs):
            if i not in used_p and j not in used_c:
                used_p.add(i)
                used_c.add(j)
                match[j] = i
        # 매칭 안 된 직전 검출 → 대기 (출구면 퇴장)
        min_hits = self.cfg["safety"].get("vanish_min_frames", 3)
        new_carry = {}
        for i, (ppos, ppx, pdist) in enumerate(self.prev):
            if i in used_p:
                continue
            if (self._prev_hits[i] if i < len(self._prev_hits) else 1) < min_hits:
                continue                              # 잠깐 깜박인 검출은 사라짐으로 보지 않는다
            in_zone = self._prev_zone[i] if i < len(self._prev_zone) else True
            box = self._prev_box[i] if i < len(self._prev_box) else None
            carried = self._prev_carry[i] if i < len(self._prev_carry) else set()
            if self._is_exit(ppx, in_zone, percept.shape) or self._box_at_edge(box, percept.shape):
                if carried:                           # 들고 있던 사람(피규어)도 함께 나갔다
                    self.pending = [pd for pd in self.pending if pd["id"] not in carried]
                continue
            self._pid += 1
            pd = {"id": self._pid, "pos": ppos, "px": ppx, "dist": pdist, "t": now, "box": box}
            self.pending.append(pd)
            # 사라진 자리에 겹쳐 있는 현재 검출(집어 든 손)을 기억한다 — 그 검출이 출구로 나가면 함께 나간 것
            if box is not None:
                for j, c in enumerate(cur):
                    if c[4] is not None and overlap_small(box, c[4]) >= 0.2:
                        new_carry.setdefault(j, set()).add(pd["id"])
        # 매칭 안 된 현재 검출 → 대기 중인 위치에서 다시 나타났나?
        for j, c in enumerate(cur):
            if j in used_c:
                continue
            for k, pd in enumerate(self.pending):
                if self._near(pd["pos"], pd["px"], c[0], c[1], pd.get("box"), c[4]):
                    self.pending.pop(k)
                    break
        # 유예시간 초과 → 미해결 사라짐
        new = []
        grace = self.cfg["safety"]["vanish_grace"]
        for pd in list(self.pending):
            if now - pd["t"] > grace:
                self.pending.remove(pd)
                self.unresolved.append(pd)
                new.append(pd)
        hits = [1] * len(cur)
        for j, i in match.items():
            hits[j] = (self._prev_hits[i] if i < len(self._prev_hits) else 1) + 1
        self._prev_hits = hits
        alive = {pd["id"] for pd in self.pending}
        carry = [set() for _ in cur]
        for j, i in match.items():
            carry[j] |= self._prev_carry[i] if i < len(self._prev_carry) else set()
        for j, ids in new_carry.items():
            carry[j] |= ids
        self._prev_carry = [c & alive for c in carry]
        self.prev = [(c[0], c[1], c[2]) for c in cur]
        self._prev_zone = [c[3] for c in cur]
        self._prev_box = [c[4] for c in cur]
        return new

    def _box_at_edge(self, box, shape):
        if box is None:
            return False
        b = self.cfg["safety"]["exit_border_px"]
        h, w = shape
        x1, y1, x2, y2 = box
        return x1 < b or x2 > w - b or y1 < b or y2 > h - b       # 네 가장자리 모두 (위·아래로 나가는 손)

    def pending_min_dist(self):
        return min((p["dist"] for p in self.pending), default=float("inf"))

    def clear(self):
        n = len(self.unresolved)
        self.unresolved.clear()
        self.pending.clear()
        self._prev_carry = [set() for _ in self._prev_carry]
        return n
