"""순환 녹화 + 이벤트 클립.

설계 (젯슨 Orin Nano에는 NVENC가 없다 → CPU 비용 최소화)
- 평상시: D435i RGB를 10fps로 JPEG 압축(libjpeg-turbo, 프레임당 수 ms)해 2초 조각(.mjpg)으로
  RAM 디스크(/dev/shm)에 쓴다. 최근 60초만 남기고 지운다 → 약 20~30MB, SSD 쓰기 없음.
- 이벤트: 전 n초 조각을 즉시 SSD 작업 폴더로 복사해 붙잡고, 후 m초 조각이 완성될 때마다 추가한 뒤
  ffmpeg(libx264 veryfast, nice)로 한 번만 H.264 MP4로 인코딩한다 → 브라우저에서 바로 재생.
- 화재처럼 끝이 정해지지 않은 녹화는 end()가 불릴 때까지 조각을 계속 모은다.
- 알림은 클립 완성을 기다리지 않는다(이벤트 허브가 즉시 보낸다).
"""
import logging
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

import cv2

log = logging.getLogger("recorder")


class Segment:
    __slots__ = ("t0", "t1", "path", "frames")

    def __init__(self, t0, path):
        self.t0, self.t1, self.path, self.frames = t0, None, path, 0


class Recorder:
    def __init__(self, camera, cfg, clip_dir):
        rc = cfg["recording"]
        self.cfg = cfg
        self.cam = camera
        self.fps = rc["fps"]
        self.seg_sec = rc["segment_sec"]
        self.keep = rc["keep_sec"]
        self.quality = rc["jpeg_quality"]
        self.seg_dir = Path(rc["segment_dir"])
        shutil.rmtree(self.seg_dir, ignore_errors=True)
        self.seg_dir.mkdir(parents=True, exist_ok=True)
        self.clip_dir = Path(clip_dir)
        self.clip_dir.mkdir(parents=True, exist_ok=True)
        # 클립 작업 폴더는 SSD에 둔다 (화재처럼 긴 녹화가 RAM을 붙잡지 않게)
        self.work_dir = self.clip_dir / "_work"
        shutil.rmtree(self.work_dir, ignore_errors=True)
        self.work_dir.mkdir(exist_ok=True)
        self.segments = []          # 완성된 조각
        self.cur = None
        self.fh = None
        self.lock = threading.Lock()
        self.jobs = {}              # key -> ClipJob
        self.enc_ms = 0.0
        self._run = True
        self.thread = threading.Thread(target=self._loop, name="recorder", daemon=True)

    def start(self):
        self.thread.start()
        return self

    # ---- 순환 녹화 ----
    def _loop(self):
        period = 1.0 / self.fps
        nxt = time.monotonic()
        while self._run:
            f = self.cam.latest()
            now = time.monotonic()
            if f is not None and now - f.t < 1.0:
                self._write(f, now)
            nxt += period
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def _write(self, f, now):
        if self.cur is None or now - self.cur.t0 >= self.seg_sec:
            self._rotate(now)
        t0 = time.monotonic()
        img = f.color
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        img = img.copy()
        cv2.putText(img, stamp, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            self.fh.write(buf.tobytes())
            self.cur.frames += 1
        self.enc_ms = 0.9 * self.enc_ms + 0.1 * (time.monotonic() - t0) * 1000

    def _rotate(self, now):
        with self.lock:
            if self.cur is not None:
                self.fh.close()
                self.cur.t1 = now
                self.segments.append(self.cur)
                for j in list(self.jobs.values()):
                    j.offer(self.cur)
            # 오래된 조각 삭제 (클립 작업은 이미 복사해 두었으므로 안전)
            while self.segments and now - self.segments[0].t1 > self.keep:
                s = self.segments.pop(0)
                try:
                    os.unlink(s.path)
                except FileNotFoundError:
                    pass
            p = self.seg_dir / f"seg_{int(time.time() * 1000)}.mjpg"
            self.cur = Segment(now, p)
            self.fh = open(p, "wb")

    # ---- 이벤트 클립 ----
    def start_clip(self, key, pre, post=None, on_done=None, name="clip"):
        """key: 이벤트 id. post=None 이면 end(key)까지 계속 녹화."""
        now = time.monotonic()
        with self.lock:
            job = ClipJob(self, key, now, pre, post, on_done, name)
            for s in self.segments:
                if s.t1 >= now - pre:
                    job.offer(s)
            self.jobs[key] = job
        job.start()
        return job

    def end(self, key):
        j = self.jobs.get(key)
        if j:
            j.stop_evt.set()

    def status(self):
        with self.lock:
            return {"segments": len(self.segments), "enc_ms": round(self.enc_ms, 1),
                    "jobs": len(self.jobs), "fps": self.fps}


class ClipJob(threading.Thread):
    def __init__(self, rec, key, t_event, pre, post, on_done, name):
        super().__init__(daemon=True, name=f"clip-{key}")
        self.rec, self.key, self.t_event = rec, key, t_event
        self.pre, self.post, self.on_done, self.name_ = pre, post, on_done, name
        self.dir = rec.work_dir / str(key)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.parts = []
        self.stop_evt = threading.Event()

    def offer(self, seg):
        dst = self.dir / seg.path.name
        if dst.exists():
            return
        try:
            shutil.copyfile(seg.path, dst)
            self.parts.append((seg.t0, dst, seg.frames))
        except FileNotFoundError:
            pass

    def run(self):
        if self.post is None:
            self.stop_evt.wait()
        else:
            deadline = self.t_event + self.post + self.rec.seg_sec + 0.5
            while time.monotonic() < deadline and not self.stop_evt.is_set():
                time.sleep(0.2)
        time.sleep(0.1)
        with self.rec.lock:
            self.rec.jobs.pop(self.key, None)
        path, size = self._encode()
        shutil.rmtree(self.dir, ignore_errors=True)
        if self.on_done:
            self.on_done(self.key, path, size)

    def _encode(self):
        parts = sorted(self.parts)
        if not parts:
            return None, 0
        cat = self.dir / "all.mjpg"
        with open(cat, "wb") as out:
            for _, p, _ in parts:
                with open(p, "rb") as f:
                    shutil.copyfileobj(f, out)
        out_path = self.rec.clip_dir / f"{time.strftime('%Y%m%d_%H%M%S')}_{self.name_}_{self.key}.mp4"
        cmd = ["nice", "-n", "10", "ffmpeg", "-loglevel", "error", "-y", "-f", "mjpeg",
               "-framerate", str(self.rec.fps), "-i", str(cat), "-c:v", "libx264", "-preset", "veryfast",
               "-crf", "28", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-threads", "2", str(out_path)]
        t0 = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            log.warning("ffmpeg 실패: %s", r.stderr[-300:])
            return None, 0
        log.info("clip %s %.1fs encode, %d parts", out_path.name, time.time() - t0, len(parts))
        return out_path, out_path.stat().st_size
