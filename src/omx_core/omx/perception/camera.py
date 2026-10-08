"""카메라 소스 (실제 D435i / 녹화 영상 파일 같은 인터페이스).

캡처 스레드가 최신 프레임 1장만 들고 있고, 소비자(인식·녹화·웹)는 필요할 때 가져간다.
pyrealsense2 프레임은 반드시 복사한다 — 복사 없이 들고 있으면 SDK 프레임 풀이 고갈돼
wait_for_frames가 수백 ms 멎는다 (~/safety_barrier 실측 교훈).
"""
import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

log = logging.getLogger("camera")


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    w: int
    h: int


@dataclass
class Frame:
    seq: int
    t: float
    color: np.ndarray            # BGR
    depth: np.ndarray = None     # uint16, 컬러에 정렬됨 (없으면 None)
    depth_scale: float = 0.001


class _Base:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.intr = None
        self.fps_meas = 0.0
        self.error = ""
        self.listeners = []          # 새 프레임마다 호출 (녹화기 등, 가볍게 유지할 것)
        self._run = True
        self.thread = threading.Thread(target=self._loop_guard, name=type(self).__name__, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self._run = False

    def latest(self):
        with self.lock:
            return self.frame

    def _publish(self, color, depth=None, scale=0.001):
        self.seq += 1
        f = Frame(self.seq, time.monotonic(), color, depth, scale)
        with self.lock:
            self.frame = f
        for cb in self.listeners:
            try:
                cb(f)
            except Exception:
                log.exception("camera listener")

    def _loop_guard(self):
        while self._run:
            try:
                self._loop()
            except Exception as e:
                self.error = str(e)
                log.warning("카메라 오류, 2초 뒤 재시도: %s", e)
                time.sleep(2.0)


class RealSenseSource(_Base):
    def __init__(self, w=640, h=480, fps=15):
        super().__init__()
        self.w, self.h, self.fps = w, h, fps

    def _loop(self):
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)
        prof = pipe.start(cfg)
        scale = prof.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        ci = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = Intrinsics(ci.fx, ci.fy, ci.ppx, ci.ppy, ci.width, ci.height)
        self.error = ""
        n, t0 = 0, time.monotonic()
        try:
            while self._run:
                fs = pipe.wait_for_frames(2000)
                fs = align.process(fs)
                c, d = fs.get_color_frame(), fs.get_depth_frame()
                if not c or not d:
                    continue
                self._publish(np.asanyarray(c.get_data()).copy(),
                              np.asanyarray(d.get_data()).copy(), scale)
                n += 1
                if time.monotonic() - t0 >= 2.0:
                    self.fps_meas = n / (time.monotonic() - t0)
                    n, t0 = 0, time.monotonic()
        finally:
            pipe.stop()


class VideoSource(_Base):
    """녹화 영상으로 시험. 깊이가 없으므로 거리 판정은 하지 않는다(2D 표시만)."""

    def __init__(self, path, fps=15, loop=True):
        super().__init__()
        self.path, self.fps, self.loop = path, fps, loop

    def _loop(self):
        cap = cv2.VideoCapture(self.path)
        if not cap.isOpened():
            raise RuntimeError(f"영상 파일 열기 실패: {self.path}")
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.intr = Intrinsics(w * 0.95, w * 0.95, w / 2, h / 2, w, h)
        period = 1.0 / self.fps
        while self._run:
            ok, img = cap.read()
            if not ok:
                if not self.loop:
                    return
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            self._publish(img)
            self.fps_meas = self.fps
            time.sleep(period)


class UsbCamSource(_Base):
    """모노 USB 카메라 (입구 헬멧). MJPEG 30fps 로 받되 압축 풀기는 fps 만큼만 한다 (grab 만 하고 버리면 CPU 가 거의 안 든다)."""

    def __init__(self, device="auto", w=640, h=480, fps=5):
        super().__init__()
        self.device, self.w, self.h, self.fps = device, w, h, fps

    @staticmethod
    def find_device(device="auto"):
        """'auto' → /dev/v4l/by-id 에서 RealSense 가 아닌 첫 카메라의 index0."""
        if device != "auto":
            return device
        import glob
        for p in sorted(glob.glob("/dev/v4l/by-id/*-video-index0")):
            if "RealSense" not in p:
                return p
        raise RuntimeError("모노 USB 카메라 없음 (/dev/v4l/by-id)")

    def _loop(self):
        import os
        dev = os.path.realpath(self.find_device(self.device))
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"카메라 열기 실패: {dev}")
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.intr = Intrinsics(w * 0.95, w * 0.95, w / 2, h / 2, w, h)
            self.error = ""
            period = 1.0 / self.fps
            next_t, n, t0 = 0.0, 0, time.monotonic()
            while self._run:
                if not cap.grab():
                    raise RuntimeError("카메라 프레임 없음")
                now = time.monotonic()
                if now < next_t:
                    continue
                next_t = now + period
                ok, img = cap.retrieve()
                if ok:
                    self._publish(img)
                    n += 1
                if now - t0 >= 2.0:
                    self.fps_meas = n / (now - t0)
                    n, t0 = 0, now
        finally:
            cap.release()


def make_camera(cfg):
    c = cfg["camera"]
    if c["source"] == "video":
        return VideoSource(c["video_path"], c["fps"])
    return RealSenseSource(c["width"], c["height"], c["fps"])
