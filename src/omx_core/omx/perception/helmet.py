"""입구 헬멧 게이트 — 모노 USB 카메라로 헬멧을 확인하면 출입문(아두이노 서보)을 연다.

설계 (PROJECT_BRIEF 3-1):
- 모노 카메라를 저프레임(기본 5fps)으로 받고, **화면에 움직임이 있을 때와 그 뒤 hold_sec(기본 3초)** 동안만 추론한다.
  (2026-10-08 실기: 피규어를 놓고 손을 떼면 바로 '움직임 없음'이 돼 확인 횟수가 지워지고 박스가 사라졌다)
- 모델: 사람 인식과 같은 통합 모델(v2: fall-down / helmet / human)을 다른 실행 통로(lane)로 같이 쓴다.
  클래스 번호는 모델마다 다르므로 이름으로 찾는다. 헬멧 판정에는 마스크가 필요 없어 만들지 않는다.
- 판정 방식 (helmet.match):
  * head (기본): 보이는 사람 모두 머리 영역(박스 위쪽)에 헬멧이 겹쳐야 착용. 사람 없이 헬멧만 보이면 열지 않는다.
  * presence: 같은 화면에 사람과 헬멧이 함께 보이면 착용 (입구 카메라 각도에서 head 가 잘 안 될 때).
  * 옛 헬멧 전용 모델(helmet/head, 사람 클래스 없음): 헬멧이 있고 맨머리(head)가 없을 때.
- 연속 N프레임 착용이어야 문 열림 요청. 미착용·불확실이면 닫힘 유지.
  마지막 확인 후 close_sec(기본 5초)가 지나면 닫는다. 문 명령 자체는 SiteController 가 보낸다.
- 결과는 문 제어에만 쓰고 안전 관리자의 로봇 거리 판단에는 섞지 않는다 (벽 너머 사람 오판 방지).
"""
import logging
import threading
import time

import cv2
import numpy as np

log = logging.getLogger("helmet")


def has_helmet(box, fallen, helmets, head_frac=0.35):
    """사람 박스의 머리 영역에 헬멧 중심이 들어오는가. 넘어진 사람은 머리 위치를 알 수 없어 박스 전체를 본다."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    if fallen:
        rx1, ry1, rx2, ry2 = x1 - 0.1 * w, y1 - 0.1 * h, x2 + 0.1 * w, y2 + 0.1 * h
    else:                                   # 헬멧이 머리 위로 조금 튀어나오는 것까지
        rx1, ry1, rx2, ry2 = x1 - 0.1 * w, y1 - 0.15 * h, x2 + 0.1 * w, y1 + head_frac * h
    for hx1, hy1, hx2, hy2 in helmets:
        cx, cy = (hx1 + hx2) / 2, (hy1 + hy2) / 2
        if rx1 <= cx <= rx2 and ry1 <= cy <= ry2:
            return True
    return False


class HelmetGate:
    def __init__(self, camera, detector, cfg, lane=0):
        self.cam, self.det, self.cfg, self.lane = camera, detector, cfg, lane
        self._names_key = None
        self.helmet_ids, self.head_ids, self.person_ids, self.fallen_ids = set(), set(), set(), set()
        self.state = "대기"
        self.error = ""
        self.open_until = 0.0
        self.last_dets = []
        self.last_ok = {}                  # 검출 index → 착용 여부 (표시용)
        self.last_frame = None
        self.last_infer_t = 0.0
        self._n = 0
        self._last_motion = -1e9
        self._run = True
        self._resolve()

    def _resolve(self):
        key = (self.det.model_name, id(self.det.names))
        if key == self._names_key:
            return
        self._names_key = key
        ids = lambda names: set(self.det.class_ids(names))  # noqa: E731
        self.helmet_ids = ids(["helmet"])
        self.head_ids = ids(["head"])
        self.person_ids = ids(["human", "person", "fall-down"])
        self.fallen_ids = ids(["fall-down"])
        if self.det.ready and not self.helmet_ids:
            log.warning("헬멧 모델에 'helmet' 클래스가 없음: %s", self.det.names)

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name="helmet").start()
        return self

    def wants_open(self, now=None):
        return (now or time.monotonic()) < self.open_until

    def _loop(self):
        prev, prev_seq = None, -1
        while self._run:
            try:
                f = self.cam.wait_new(prev_seq, timeout=0.5)
                if f is None:
                    if self.cam.latest() is None:
                        self.state = "카메라 없음" + (f" ({self.cam.error})" if self.cam.error else "")
                    continue
                prev_seq = f.seq
                self.last_frame = f.color
                small = cv2.cvtColor(f.color[::8, ::8], cv2.COLOR_BGR2GRAY).astype(np.int16)
                # 작은 피규어도 잡히게 평균 차이가 아니라 '바뀐 화소 비율'로 본다 (80x60 중 0.5% ≈ 24화소)
                moving = prev is None or np.count_nonzero(np.abs(small - prev) > 20) > 0.005 * small.size
                prev = small
                now = time.monotonic()
                if moving:
                    self._last_motion = now
                # 움직임이 멈춘 뒤에도 잠시 더 본다: 들어와서 멈춰 선 사람(피규어)도 연속 확인이 끝나게
                if now - self._last_motion > self.cfg["helmet"].get("hold_sec", 3.0):
                    self._n = 0
                    self.last_dets = []
                    if not self.wants_open():
                        self.state = "대기 (움직임 없음)"
                    continue
                self._resolve()
                self.evaluate(self.det.infer(f.color, lane=self.lane, masks=False), time.monotonic())
                self.error = ""
            except Exception as e:                  # 한 번의 오류로 스레드가 죽지 않게. 문은 닫힘 유지(마지막 확인 후 close_sec)
                self._n = 0
                self.error = str(e)
                self.state = f"오류 — 닫힘 유지 ({e})"
                log.exception("helmet")
                time.sleep(0.5)

    def evaluate(self, dets, now):
        h = self.cfg["helmet"]
        self.last_dets = dets
        self.last_infer_t = now
        helmets = [d.box for d in dets if d.cls in self.helmet_ids and d.conf >= h["conf"]]
        self.last_ok = {}
        missing = False
        if self.person_ids:
            pconf = h.get("person_conf", h["conf"])
            persons = [(i, d) for i, d in enumerate(dets) if d.cls in self.person_ids and d.conf >= pconf]
            if h.get("match", "head") == "presence":
                ok = bool(persons) and bool(helmets)
                for i, _ in persons:
                    self.last_ok[i] = ok
            else:
                for i, d in persons:
                    self.last_ok[i] = has_helmet(d.box, d.cls in self.fallen_ids, helmets, h.get("head_frac", 0.35))
                missing = not all(self.last_ok.values())
                ok = bool(persons) and not missing
        else:                                       # 옛 헬멧 전용 모델 (helmet/head)
            missing = any(d.cls in self.head_ids for d in dets)
            ok = bool(helmets) and not missing
        self._n = self._n + 1 if ok else 0
        if self._n >= h["confirm_frames"]:
            if not self.wants_open(now):
                log.info("헬멧 확인 — 출입문 열림")
            self.open_until = now + self.cfg["door"]["close_sec"]
            self.state = "헬멧 확인 — 문 열림"
        elif missing:
            self.state = "헬멧 미착용 — 닫힘 유지"
        elif ok:
            self.state = f"헬멧 확인 중 ({self._n}/{h['confirm_frames']})"
        elif helmets:
            self.state = "사람 확인 중 (헬멧만 보임)"
        else:
            self.state = "사람 확인 중" if dets else "헬멧 없음"

    def overlay(self):
        img = self.last_frame
        if img is None:
            return None
        img = img.copy()
        fresh = time.monotonic() - self.last_infer_t < 1.0
        for i, d in enumerate(self.last_dets if fresh else []):
            if d.cls in self.helmet_ids:
                ok, label = True, "helmet"
            elif i in self.last_ok:
                ok = self.last_ok[i]
                label = f"{d.name} {'OK' if ok else 'NO HELMET'}"
            else:
                ok, label = d.cls not in self.head_ids, d.name
            c = (80, 200, 60) if ok else (40, 40, 230)
            x1, y1, x2, y2 = (int(v) for v in d.box)
            cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
            cv2.putText(img, f"{label} {d.conf:.2f}", (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        return img

    def snapshot(self):
        rem = self.open_until - time.monotonic()
        return {"state": self.state, "door_wanted": rem > 0, "open_left": round(max(0.0, rem), 1),
                "camera_fps": round(self.cam.fps_meas, 1), "camera_error": self.cam.error,
                "model": self.det.model_name, "infer_ms": round(self.det.lane_ms[self.lane], 1),
                "match": self.cfg["helmet"].get("match", "head"), "error": self.error,
                "detections": [{"name": d.name, "conf": round(d.conf, 2)} for d in self.last_dets]}
