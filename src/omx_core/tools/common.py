"""시험 도구 공용: 로봇 연결 + 무인 시험용 사람 감시(카메라에 사람이 보이면 허가 끊음)."""
import csv
import logging
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.config import Config, ROOT  # noqa: E402
from omx.robot.dxl_bus import DxlBus, SimBus  # noqa: E402
from omx.robot.executor import Executor  # noqa: E402

LOGDIR = ROOT / "reports" / "overnight"
LOGDIR.mkdir(parents=True, exist_ok=True)


def make_executor(sim=False, on_event=None):
    C = Config()
    r = C["robot"]
    ids = [r["ids"][k] for k in ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")]
    if sim:
        bus = SimBus()
    else:
        bus = DxlBus(r["port"], r["baud"], ids, C.limits["gripper_current_ma"])
    ex = Executor(bus, r, C.limits, on_event=on_event or (lambda k, m: print(f"[event] {k}: {m}")))
    ex.start()
    return C, ex


class PersonGuard:
    """카메라에 사람이 near_m 안에 보이면(또는 카메라가 멎으면) 허가 0. 20Hz로 permit 전송."""

    def __init__(self, ex, C, near_m=1.5):
        from omx.perception.camera import RealSenseSource
        from omx.perception.detector import Detector, det_mask
        self.ex, self.near_m, self.det_mask = ex, near_m, det_mask
        self.cam = RealSenseSource(640, 480, 15).start()
        self.det = Detector(C.data_dir, C["perception"]["model"], conf=C["perception"]["conf"])
        self.person = False
        self.last_ok = 0.0
        self.note = ""
        self.frame = None
        threading.Thread(target=self._vision, daemon=True).start()
        threading.Thread(target=self._permit, daemon=True).start()

    def _vision(self):
        last_seq = -1
        while True:
            f = self.cam.latest()
            if f is None or f.seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = f.seq
            self.frame = f
            dets = self.det.infer(f.color, classes=[0])
            near = False
            for d in dets:
                m = self.det_mask(d, f.color.shape)
                z = f.depth[m].astype(float) * f.depth_scale
                z = z[(z > 0.1) & (z < 6)]
                dist = float(np.median(z)) if len(z) > 30 else 0.0      # 깊이를 모르면 가깝다고 본다
                if dist < self.near_m:
                    near = True
                    self.note = f"사람 감지 conf={d.conf:.2f} 거리={dist:.2f}m"
            self.person = near
            self.last_ok = time.monotonic()

    def _permit(self):
        while True:
            stale = time.monotonic() - self.last_ok > 1.5
            if stale:
                self.ex.permit(0.0, "카메라/인식 멎음")
            elif self.person:
                self.ex.permit(0.0, self.note)
            else:
                self.ex.permit(1.0, "OK")
            time.sleep(0.05)

    def wait_ready(self, timeout=30):
        t0 = time.time()
        while time.monotonic() - self.last_ok > 1.0:
            if time.time() - t0 > timeout:
                raise RuntimeError("카메라/인식 준비 안 됨")
            time.sleep(0.2)

    def snap(self, path):
        import cv2
        if self.frame is not None:
            cv2.imwrite(str(path), self.frame.color)


class StateLogger:
    def __init__(self, ex, path, hz=10):
        self.ex = ex
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t"] + [f"goal{i}" for i in range(6)] + [f"q{i}" for i in range(6)]
                        + [f"temp{i}" for i in range(6)] + [f"load{i}" for i in range(6)] + ["scale", "fault"])
        self.t0 = time.monotonic()
        self.hz = hz
        self.max_err = np.zeros(6)
        self.max_temp = np.zeros(6)
        self.max_load = np.zeros(6)
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._run:
            ex, st = self.ex, self.ex.state
            if st is not None and ex.goal is not None:
                g = ex.goal.copy()
                err = np.abs(g - st.q)
                self.max_err = np.maximum(self.max_err, err)
                self.max_temp = np.maximum(self.max_temp, st.temp)
                self.max_load = np.maximum(self.max_load, st.load)
                self.w.writerow([f"{time.monotonic() - self.t0:.2f}"] + [f"{v:.4f}" for v in g]
                                + [f"{v:.4f}" for v in st.q] + [int(v) for v in st.temp]
                                + [f"{v:.2f}" for v in st.load] + [ex.scale_now, ex.fault])
            time.sleep(1 / self.hz)

    def close(self):
        self._run = False
        time.sleep(0.2)
        self.f.close()


def wait_motion(m, ex, timeout=60):
    st = m.wait(timeout)
    if st != "done":
        print(f"  !! motion {m.name}: {st} {m.reason} fault={ex.fault}")
    return st == "done"


def park_and_release(ex, log=print):
    """휴식 자세로 천천히 옮긴 뒤 토크를 끄고, 떨어지지 않는지 2초간 확인."""
    rest = np.array(list(ex.cfg["rest_pose"]) + [ex.current_q()[5]])
    rest[0] = ex.current_q()[0]          # 베이스는 돌리지 않는다 (접힌 채로 제자리)
    m = ex.move_joints(rest, "rest")
    if not wait_motion(m, ex, 90):
        log("휴식 자세 이동 실패 — 토크 유지")
        return False
    time.sleep(0.8)
    q_before = ex.state.q.copy()
    ex.set_torque(False)
    time.sleep(2.0)
    drop = np.abs(ex.state.q[:5] - q_before[:5]).max()
    log(f"토크 끈 뒤 2초간 최대 변화 {drop:.3f} rad")
    if drop > 0.1:
        log("팔이 처짐 → 토크 다시 켬")
        ex.set_torque(True)
        return False
    return True
