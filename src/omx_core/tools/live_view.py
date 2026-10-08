"""v2 모델 실시간 확인 뷰어 — 작업대(D435i)와 입구 카메라 영상에 검출 결과를 그려 한 창에 보여준다.

  ~/ros2_ws/.venv/bin/python ~/ros2_ws/src/omx_core/tools/live_view.py [--conf 0.25] [--no-mono]

- 모든 클래스(human / fall-down / helmet)를 확신도와 함께 그린다. 시스템 판정 기준보다 낮은 것은 얇은 선으로.
  (사람 0.60, 입구 사람·헬멧 0.50 — config/default.yaml)
- D435i 는 사람 박스 가운데의 깊이(m)도 표시한다.
- 키: q 종료, s 화면 저장(~/omx_view_시각.jpg), +/- 표시 최소 확신도 조절
- ROS 시스템(또는 데모)과 동시에 실행하지 않는다 (같은 카메라를 잡는다).
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from omx.perception.gpu_post import blocking_sync  # noqa: E402

blocking_sync()
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyrealsense2 as rs  # noqa: E402

from omx.perception.detector import Detector  # noqa: E402

COL = {"human": (0, 200, 0), "fall-down": (0, 0, 255), "helmet": (255, 200, 0)}
MONO = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0"
SYS_CONF = {"human": 0.60, "fall-down": 0.60, "helmet": 0.50}


def draw(img, dets, conf_min, depth=None, title=""):
    img = img.copy()
    for d in dets:
        if d.conf < conf_min:
            continue
        x1, y1, x2, y2 = map(int, d.box)
        c = COL.get(d.name, (200, 200, 200))
        thick = 2 if d.conf >= SYS_CONF.get(d.name, 0.5) else 1     # 시스템 기준 미만은 얇게
        cv2.rectangle(img, (x1, y1), (x2, y2), c, thick)
        label = f"{d.name} {d.conf:.2f}"
        if depth is not None and d.name != "helmet":
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            roi = depth[max(0, cy - 3):cy + 4, max(0, cx - 3):cx + 4]
            v = roi[roi > 0]
            if v.size:
                label += f" {np.median(v) / 1000:.2f}m"
        cv2.putText(img, label, (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2 if thick == 2 else 1)
    cv2.putText(img, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.25, help="이 확신도 이상만 그린다")
    ap.add_argument("--no-mono", action="store_true", help="입구 카메라를 쓰지 않음")
    ap.add_argument("--model", default="v2_best.engine")
    a = ap.parse_args()
    if not os.environ.get("DISPLAY"):
        sys.exit("화면(DISPLAY)이 없음 — 원격 데스크톱(XFCE) 안의 터미널에서 실행하세요")
    det = Detector(Path.home() / ".cache" / "omx_view", a.model, 0.10, lanes=2, require_any=["human", "person"])
    if not det.ready:
        sys.exit(f"모델 불러오기 실패: {det.status}")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    mono = None
    if not a.no_mono and os.path.exists(MONO):
        mono = cv2.VideoCapture(os.path.realpath(MONO), cv2.CAP_V4L2)
        mono.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        mono.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        mono.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        mono.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    conf_min = a.conf
    win = "OMX v2 live view (q: quit, s: save, +/-: conf)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    n, t0, fps = 0, time.monotonic(), 0.0
    try:
        while True:
            fs = align.process(pipe.wait_for_frames())
            color = np.asanyarray(fs.get_color_frame().get_data()).copy()
            depth = np.asanyarray(fs.get_depth_frame().get_data()).copy()
            d1 = det.infer(color, None, lane=0, masks=False)
            left = draw(color, d1, conf_min, depth, f"D435i  {det.lane_ms[0]:.0f}ms  {fps:.0f}fps")
            if mono is not None:
                ok, img = mono.read()
                if ok:
                    d2 = det.infer(img, None, lane=1, masks=False)
                    right = draw(img, d2, conf_min, None, f"Entrance  {det.lane_ms[1]:.0f}ms")
                else:
                    right = np.zeros_like(left)
                view = np.hstack([left, cv2.resize(right, (left.shape[1], left.shape[0]))])
            else:
                view = left
            cv2.putText(view, f"min conf {conf_min:.2f}  (thick = above system threshold)", (8, view.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(win, view)
            n += 1
            if time.monotonic() - t0 >= 1.0:
                fps, n, t0 = n / (time.monotonic() - t0), 0, time.monotonic()
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q") or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord("s"):
                p = Path.home() / f"omx_view_{time.strftime('%H%M%S')}.jpg"
                cv2.imwrite(str(p), view)
                print("저장:", p)
            if k in (ord("+"), ord("=")):
                conf_min = min(0.95, conf_min + 0.05)
            if k == ord("-"):
                conf_min = max(0.05, conf_min - 0.05)
    finally:
        pipe.stop()
        if mono is not None:
            mono.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
