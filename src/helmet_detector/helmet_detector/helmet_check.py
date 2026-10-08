import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import time
import os
import subprocess
import threading
import collections
import requests
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from ultralytics import YOLO

BACKEND_URL = "http://192.168.137.1:8000"
USERNAME = "admin"
PASSWORD = ""  # 비밀번호는 코드에 두지 않음 (공개 저장소) — 쓸 때 직접 넣기
PRE_ROLL_SEC = 5
POST_ROLL_SEC = 15
RECORD_FPS = 10

# v2 통합 모델 클래스: 0=fall-down, 1=helmet, 2=human
CLS_FALL = 0
CLS_HELMET = 1
CLS_HUMAN = 2
HEAD_RATIO = 0.5  # 서 있는 사람은 박스 위쪽 이 비율 안에 안전모 중심이 있어야 착용으로 판단


def has_helmet(person_box, person_cls, helmet_boxes):
    x1, y1, x2, y2 = person_box
    # 넘어진 사람은 머리 위치를 알 수 없으므로 박스 전체를 본다
    y_limit = y2 if person_cls == CLS_FALL else y1 + (y2 - y1) * HEAD_RATIO
    for hx1, hy1, hx2, hy2 in helmet_boxes:
        cx, cy = (hx1 + hx2) / 2.0, (hy1 + hy2) / 2.0
        if x1 <= cx <= x2 and y1 <= cy <= y_limit:
            return True
    return False


class EventRecorder:
    def __init__(self, logger):
        self.logger = logger
        self.token = None
        self.buffer = collections.deque()
        self.lock = threading.Lock()
        self.recording = False
        self.record_frames = []
        self.record_until = 0
        self.pending_meta = None
        self.login()

    def login(self):
        try:
            res = requests.post(f'{BACKEND_URL}/api/auth/login',
                                 json={'username': USERNAME, 'password': PASSWORD}, timeout=8)
            res.raise_for_status()
            self.token = res.json()['token']
        except Exception as e:
            self.logger.warn(f'[녹화] 백엔드 로그인 실패: {e}')
            self.token = None

    def push_frame(self, frame):
        now = time.time()
        with self.lock:
            self.buffer.append((now, frame.copy()))
            while self.buffer and now - self.buffer[0][0] > PRE_ROLL_SEC + 0.5:
                self.buffer.popleft()
            if self.recording:
                self.record_frames.append(frame.copy())
                if now >= self.record_until:
                    frames = self.record_frames
                    meta = self.pending_meta
                    self.recording = False
                    self.record_frames = []
                    threading.Thread(target=self._save_and_upload, args=(frames, meta), daemon=True).start()

    def trigger(self, event_type, title, detail):
        with self.lock:
            if self.recording:
                return
            self.recording = True
            self.record_frames = [f for (t, f) in self.buffer]
            self.record_until = time.time() + POST_ROLL_SEC
            self.pending_meta = (event_type, title, detail)
        self.logger.warn(f'[녹화] 이벤트 트리거: {title} — 녹화 시작')

    def _save_and_upload(self, frames, meta):
        event_type, title, detail = meta
        if not frames:
            return
        raw_path = f'/tmp/event_raw_{int(time.time())}.mp4'
        final_path = f'/tmp/event_{int(time.time())}.mp4'
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(raw_path, cv2.VideoWriter_fourcc(*'mp4v'), RECORD_FPS, (w, h))
        for f in frames:
            writer.write(f)
        writer.release()

        upload_path = raw_path
        try:
            result = subprocess.run(
                ['ffmpeg', '-y', '-i', raw_path, '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                 '-movflags', '+faststart', final_path],
                capture_output=True, timeout=30)
            if result.returncode == 0 and os.path.exists(final_path):
                upload_path = final_path
            else:
                self.logger.warn(f'[녹화] ffmpeg 변환 실패, 원본으로 시도: {result.stderr.decode(errors="ignore")[:200]}')
        except Exception as e:
            self.logger.warn(f'[녹화] ffmpeg 실행 실패: {e}')

        if self.token is None:
            self.login()
        if self.token is None:
            self.logger.warn('[녹화] 로그인 안 되어 업로드 취소')
            for p in (raw_path, final_path):
                try:
                    os.remove(p)
                except OSError:
                    pass
            return
        try:
            res = requests.post(f'{BACKEND_URL}/api/events',
                json={'type': event_type, 'title': title, 'detail': detail,
                      'duration': f'{PRE_ROLL_SEC}s+{POST_ROLL_SEC}s'},
                headers={'Authorization': f'Bearer {self.token}'}, timeout=8)
            res.raise_for_status()
            event_id = res.json()['id']
            with open(upload_path, 'rb') as f:
                requests.post(f'{BACKEND_URL}/api/events/{event_id}/video',
                    files={'file': ('event.mp4', f, 'video/mp4')},
                    headers={'Authorization': f'Bearer {self.token}'}, timeout=15)
            self.logger.info(f'[녹화] 이벤트 #{event_id} 영상 업로드 완료')
        except Exception as e:
            self.logger.warn(f'[녹화] 업로드 실패: {e}')
        finally:
            for p in (raw_path, final_path):
                try:
                    os.remove(p)
                except OSError:
                    pass


class MjpegHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        target_interval = 1.0 / 12
        try:
            while True:
                t0 = time.time()
                frame = self.server.get_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if not ok:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b'\r\n')
                elapsed = time.time() - t0
                if elapsed < target_interval:
                    time.sleep(target_interval - elapsed)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class MjpegServer:
    def __init__(self, port):
        self.latest_frame = None
        self.lock = threading.Lock()
        self.server = ThreadedHTTPServer(('0.0.0.0', port), MjpegHandler)
        self.server.get_frame = self.get_frame
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def update(self, frame):
        with self.lock:
            self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()


class HelmetDetector(Node):
    def __init__(self):
        super().__init__('helmet_detector')
        self.get_logger().info('모델 로딩 중...')
        self.model = YOLO('/home/user/models/v2_best.engine', task='segment')
        self.get_logger().info('모델 로딩 완료')

        self.cap = cv2.VideoCapture('/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0')
        if not self.cap.isOpened():
            self.get_logger().error('웹캠을 열 수 없습니다')

        self.mjpeg = MjpegServer(8082)
        self.recorder = EventRecorder(self.get_logger())
        self.last_helmet_ok = True
        self.get_logger().info('스트리밍 서버 시작: http://0.0.0.0:8082/stream')

        self.publisher_ = self.create_publisher(Bool, '/helmet_ok', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, verbose=False, conf=0.5)
        boxes = results[0].boxes
        debug_img = frame.copy()

        if boxes is not None and len(boxes) > 0:
            helmets, persons = [], []
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i])
                conf = float(boxes.conf[i])
                box = tuple(boxes.xyxy[i].cpu().numpy().astype(int))
                if cls_id == CLS_HELMET:
                    helmets.append(box)
                    x1, y1, x2, y2 = box
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 200, 0), 1)
                else:
                    persons.append((box, cls_id, conf))

            # 사람 없이 안전모만 보이는 경우(근접 촬영 등)는 착용으로 간주
            overall_ok = True
            for (x1, y1, x2, y2), cls_id, conf in persons:
                ok = has_helmet((x1, y1, x2, y2), cls_id, helmets)
                if not ok:
                    overall_ok = False

                color = (0, 255, 0) if ok else (0, 0, 255)
                label = 'helmet' if ok else 'no helmet'
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug_img, f'{label} {conf:.2f}', (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            out = Bool()
            out.data = overall_ok
            self.publisher_.publish(out)

            if self.last_helmet_ok and not overall_ok:
                self.recorder.trigger('HELMET', '안전모 미착용 감지',
                                       '출입구 모노카메라 - 미착용 확인, 출입문 잠금')
            self.last_helmet_ok = overall_ok

        self.mjpeg.update(debug_img)
        self.recorder.push_frame(debug_img)

def main(args=None):
    rclpy.init(args=args)
    node = HelmetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
