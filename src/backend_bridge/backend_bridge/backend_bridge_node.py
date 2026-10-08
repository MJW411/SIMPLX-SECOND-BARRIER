import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import requests
import time

BACKEND_URL = "http://192.168.137.1:8000"
USERNAME = "admin"
PASSWORD = ""  # 비밀번호는 코드에 두지 않음 (공개 저장소) — 쓸 때 직접 넣기

class BackendBridge(Node):
    def __init__(self):
        super().__init__('backend_bridge')
        self.token = None
        self.latest_state = 'NORMAL'
        self.latest_distance = None
        self.latest_helmet_ok = None
        self.latest_x = None
        self.latest_y = None
        self.last_login_attempt = 0

        self.create_subscription(String, '/risk_state', self.state_cb, 10)
        self.create_subscription(Float32, '/distance', self.distance_cb, 10)
        self.create_subscription(Bool, '/helmet_ok', self.helmet_cb, 10)
        self.create_subscription(Float32, '/object_x', self.x_cb, 10)
        self.create_subscription(Float32, '/object_y', self.y_cb, 10)

        self.safe_distance_pub = self.create_publisher(Float32, '/cfg/safe_distance', 10)
        self.escalate_ms_pub = self.create_publisher(Float32, '/cfg/escalate_ms', 10)

        self.login()
        self.timer = self.create_timer(0.5, self.push_status)
        self.settings_timer = self.create_timer(3.0, self.fetch_settings)

    def login(self):
        try:
            res = requests.post(f'{BACKEND_URL}/api/auth/login',
                                 json={'username': USERNAME, 'password': PASSWORD}, timeout=8)
            res.raise_for_status()
            self.token = res.json()['token']
            self.get_logger().info('백엔드 로그인 성공')
        except Exception as e:
            self.get_logger().error(f'백엔드 로그인 실패: {e}')

    def state_cb(self, msg): self.latest_state = msg.data
    def distance_cb(self, msg): self.latest_distance = msg.data
    def helmet_cb(self, msg): self.latest_helmet_ok = msg.data
    def x_cb(self, msg): self.latest_x = msg.data
    def y_cb(self, msg): self.latest_y = msg.data

    def fetch_settings(self):
        if self.token is None:
            return
        try:
            res = requests.get(f'{BACKEND_URL}/api/settings',
                                headers={'Authorization': f'Bearer {self.token}'}, timeout=8)
            res.raise_for_status()
            data = res.json()
            sd = Float32()
            sd.data = float(data['safe_distance'])
            self.safe_distance_pub.publish(sd)
            em = Float32()
            em.data = float(data['escalate_ms'])
            self.escalate_ms_pub.publish(em)
        except Exception as e:
            self.get_logger().warn(f'설정 조회 실패: {e}')

    def push_status(self):
        if self.token is None:
            now = time.time()
            if now - self.last_login_attempt < 5:
                return
            self.last_login_attempt = now
            self.login()
            return
        payload = {'state': self.latest_state}
        if self.latest_distance is not None:
            payload['distance'] = self.latest_distance
        if self.latest_helmet_ok is not None:
            payload['helmet_ok'] = self.latest_helmet_ok
        if self.latest_x is not None:
            payload['x_pos'] = self.latest_x
        if self.latest_y is not None:
            payload['y_pos'] = self.latest_y
        try:
            res = requests.post(
                f'{BACKEND_URL}/api/status', json=payload,
                headers={'Authorization': f'Bearer {self.token}'}, timeout=8)
            if res.status_code == 401:
                self.get_logger().warn('토큰 만료, 재로그인')
                self.token = None
                self.login()
        except Exception as e:
            self.get_logger().warn(f'백엔드 전송 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BackendBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
