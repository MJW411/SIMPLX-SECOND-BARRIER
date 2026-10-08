"""입구 헬멧 게이트 — 모노 USB 카메라로 헬멧을 확인하면 출입문(아두이노 서보)을 연다.

설계 (PROJECT_BRIEF 3-1):
- 모노 카메라를 저프레임(기본 5fps)으로 받고, **화면에 움직임이 있을 때만** 추론한다.
- 헬멧 검출 모델: ~/models/helmet_best_fixed.engine (0=helmet, 1=head). helmet_best.* 는 클래스 이름이 뒤바뀐
  원본이라 쓰지 않는다. 클래스 번호는 모델마다 다르므로 이름으로 찾는다.
- 연속 N프레임 helmet 이고 head 가 없을 때만 문 열림 요청. 미착용·불확실이면 닫힘 유지.
  마지막 확인 후 close_sec(기본 5초)가 지나면 닫는다. 문 명령 자체는 SiteController 가 보낸다.
- 결과는 문 제어에만 쓰고 안전 관리자의 로봇 거리 판단에는 섞지 않는다 (벽 너머 사람 오판 방지).
"""
import logging
import threading
import time

import cv2
import numpy as np

log = logging.getLogger("helmet")


class HelmetGate:
    def __init__(self, camera, detector, cfg):
        self.cam, self.det, self.cfg = camera, detector, cfg
        inv = {v: k for k, v in detector.names.items()}
        self.helmet_cls, self.head_cls = inv.get("helmet"), inv.get("head")
        if self.helmet_cls is None:
            log.warning("헬멧 모델에 'helmet' 클래스가 없음: %s", detector.names)
        self.state = "대기"
        self.open_until = 0.0
        self.last_dets = []
        self.last_frame = None
        self.last_infer_t = 0.0
        self._n = 0
        self._run = True

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name="helmet").start()
        return self

    def wants_open(self, now=None):
        return (now or time.monotonic()) < self.open_until

    def _loop(self):
        prev, prev_seq = None, -1
        while self._run:
            f = self.cam.latest()
            if f is None or f.seq == prev_seq:
                if f is None:
                    self.state = "카메라 없음" + (f" ({self.cam.error})" if self.cam.error else "")
                time.sleep(0.05)
                continue
            prev_seq = f.seq
            self.last_frame = f.color
            small = cv2.cvtColor(f.color[::8, ::8], cv2.COLOR_BGR2GRAY).astype(np.int16)
            # 작은 피규어도 잡히게 평균 차이가 아니라 '바뀐 화소 비율'로 본다 (80x60 중 0.5% ≈ 24화소)
            moving = prev is None or np.count_nonzero(np.abs(small - prev) > 20) > 0.005 * small.size
            prev = small
            if not moving:                      # 움직임이 없으면 추론하지 않는다 (확인 횟수는 처음부터)
                self._n = 0
                self.last_dets = []
                if not self.wants_open():
                    self.state = "대기 (움직임 없음)"
                continue
            self.evaluate(self.det.infer(f.color), time.monotonic())

    def evaluate(self, dets, now):
        h = self.cfg["helmet"]
        self.last_dets = dets
        self.last_infer_t = now
        helmet = any(d.cls == self.helmet_cls and d.conf >= h["conf"] for d in dets)
        head = any(d.cls == self.head_cls for d in dets)
        self._n = self._n + 1 if helmet and not head else 0
        if self._n >= h["confirm_frames"]:
            if not self.wants_open(now):
                log.info("헬멧 확인 — 출입문 열림")
            self.open_until = now + self.cfg["door"]["close_sec"]
            self.state = "헬멧 확인 — 문 열림"
        elif head:
            self.state = "헬멧 미착용 — 닫힘 유지"
        elif helmet:
            self.state = f"헬멧 확인 중 ({self._n}/{h['confirm_frames']})"
        else:
            self.state = "사람 확인 중" if dets else "헬멧 없음"

    def overlay(self):
        img = self.last_frame
        if img is None:
            return None
        img = img.copy()
        fresh = time.monotonic() - self.last_infer_t < 1.0
        for d in (self.last_dets if fresh else []):
            ok = d.cls == self.helmet_cls
            c = (80, 200, 60) if ok else (40, 40, 230)
            x1, y1, x2, y2 = (int(v) for v in d.box)
            cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
            label = f"{'helmet' if ok else d.name} {d.conf:.2f}"
            cv2.putText(img, label, (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
        return img

    def snapshot(self):
        rem = self.open_until - time.monotonic()
        return {"state": self.state, "door_wanted": rem > 0, "open_left": round(max(0.0, rem), 1),
                "camera_fps": round(self.cam.fps_meas, 1), "camera_error": self.cam.error,
                "model": self.det.model_name, "infer_ms": round(self.det.infer_ms, 1),
                "detections": [{"name": d.name, "conf": round(d.conf, 2)} for d in self.last_dets]}
