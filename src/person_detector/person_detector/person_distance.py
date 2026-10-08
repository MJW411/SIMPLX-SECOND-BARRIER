import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, String, Bool
from cv_bridge import CvBridge
import numpy as np
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
FALL_CONFIRM_FRAMES = 5  # 연속 N프레임 fall-down 감지 시 낙상 확정


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
        if frame.shape[1] > 640:
            scale = 640 / frame.shape[1]
            frame = cv2.resize(frame, (640, int(frame.shape[0] * scale)))
        with self.lock:
            self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()


class PersonDistance(Node):
    def __init__(self):
        super().__init__('person_distance')
        self.bridge = CvBridge()
        self.get_logger().info('모델 로딩 중...')
        self.model = YOLO('/home/user/models/v2_best.engine', task='segment')
        self.get_logger().info('모델 로딩 완료')
        self.conf = self.declare_parameter('conf', 0.5).value
        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = None
        self.last_risk_state = 'NORMAL'
        self.latest_distance = None
        self.fall_frames = 0
        self.fall_active = False

        self.mjpeg = MjpegServer(8081)
        self.recorder = EventRecorder(self.get_logger())
        self.get_logger().info('스트리밍 서버 시작: http://0.0.0.0:8081/stream')

        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.risk_sub = self.create_subscription(
            String, '/risk_state', self.risk_callback, 10)

        self.publisher_ = self.create_publisher(Float32, '/distance', 10)
        self.x_publisher_ = self.create_publisher(Float32, '/object_x', 10)
        self.y_publisher_ = self.create_publisher(Float32, '/object_y', 10)
        self.fall_publisher_ = self.create_publisher(Bool, '/fall_detected', 10)

    def info_callback(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def risk_callback(self, msg):
        new_state = msg.data
        if self.last_risk_state != 'EMERGENCY' and new_state == 'EMERGENCY':
            d = self.latest_distance if self.latest_distance is not None else 0.0
            self.recorder.trigger('HUMAN', '로봇 접근 감지 — 강제정지',
                                   f'작업자 {d:.2f}m 접근으로 로봇 정지')
        self.last_risk_state = new_state

    def update_fall(self, fall_present):
        self.fall_frames = self.fall_frames + 1 if fall_present else 0
        fall_active = self.fall_frames >= FALL_CONFIRM_FRAMES
        if fall_active and not self.fall_active:
            self.get_logger().error('낙상 감지')
            self.recorder.trigger('FALL', '작업자 낙상 감지',
                                   f'{FALL_CONFIRM_FRAMES}프레임 연속 쓰러짐 확인')
        elif not fall_active and self.fall_active:
            self.get_logger().info('낙상 상태 해제')
        self.fall_active = fall_active
        out = Bool()
        out.data = fall_active
        self.fall_publisher_.publish(out)

    def color_callback(self, msg):
        if self.latest_depth is None:
            return
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(color_image, verbose=False, conf=self.conf)

        boxes = results[0].boxes
        debug_img = color_image.copy()

        fall_present = boxes is not None and any(int(c) == CLS_FALL for c in boxes.cls)
        self.update_fall(fall_present)
        if self.fall_active:
            cv2.putText(debug_img, 'FALL DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        if boxes is None or len(boxes) == 0:
            out = Float32()
            out.data = -1.0
            self.publisher_.publish(out)
            self.mjpeg.update(debug_img)
            self.recorder.push_frame(debug_img)
            return

        h, w = self.latest_depth.shape
        candidates = []

        for i in range(len(boxes)):
            cls_i = int(boxes.cls[i])
            # 거리 계산은 사람(서 있음/넘어짐)만 대상, 안전모는 제외
            if cls_i == CLS_HELMET:
                continue
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            conf_i = float(boxes.conf[i])
            mask_poly = None
            if results[0].masks is not None and i < len(results[0].masks.xy):
                poly = results[0].masks.xy[i].astype(np.int32)
                if len(poly) >= 3:
                    mask_poly = poly

            if mask_poly is not None:
                mask = np.zeros((h, w), dtype=np.uint8)
                cv2.fillPoly(mask, [mask_poly], 255)
                region = self.latest_depth[mask > 0]
                M = cv2.moments(mask, binaryImage=True)
                px = M['m10'] / M['m00'] if M['m00'] > 0 else (x1 + x2) / 2.0
                py = M['m01'] / M['m00'] if M['m00'] > 0 else (y1 + y2) / 2.0
            else:
                x1c, x2c = max(0, x1), min(w, x2)
                y1c, y2c = max(0, y1), min(h, y2)
                region = self.latest_depth[y1c:y2c, x1c:x2c]
                px, py = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            valid = region[region > 0]
            if len(valid) == 0:
                continue
            distance_m = float(np.median(valid)) / 1000.0
            candidates.append((distance_m, px, py, mask_poly, (x1, y1, x2, y2), conf_i, cls_i))

        if not candidates:
            out = Float32()
            out.data = -1.0
            self.publisher_.publish(out)
            self.mjpeg.update(debug_img)
            self.recorder.push_frame(debug_img)
            return

        for distance_m, px, py, mask_poly, (x1, y1, x2, y2), conf_i, cls_i in candidates:
            color = (0, 0, 255) if cls_i == CLS_FALL else (0, 180, 0)
            if mask_poly is not None:
                cv2.polylines(debug_img, [mask_poly], True, color, 1)
            else:
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 1)
            if cls_i == CLS_FALL:
                cv2.putText(debug_img, f'fall {conf_i:.2f}', (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        distance_m, px, py, mask_poly, (x1, y1, x2, y2), conf_val, _ = min(candidates, key=lambda c: c[0])

        if mask_poly is not None:
            overlay = debug_img.copy()
            cv2.fillPoly(overlay, [mask_poly], (0, 255, 0))
            debug_img = cv2.addWeighted(overlay, 0.35, debug_img, 0.65, 0)
            cv2.polylines(debug_img, [mask_poly], True, (0, 255, 0), 2)
        else:
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.latest_distance = distance_m
        out = Float32()
        out.data = distance_m
        self.publisher_.publish(out)

        if self.fx is not None:
            x_pos = (px - self.cx) * distance_m / self.fx
            y_pos = (py - self.cy) * distance_m / self.fy
            xm, ym = Float32(), Float32()
            xm.data, ym.data = x_pos, y_pos
            self.x_publisher_.publish(xm)
            self.y_publisher_.publish(ym)

        self.get_logger().info(f'감지 {len(candidates)}명 중 최근접: {distance_m:.2f}m conf={conf_val:.2f}')
        self.mjpeg.update(debug_img)
        self.recorder.push_frame(debug_img)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
