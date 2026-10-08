import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time

class RiskJudge(Node):
    def __init__(self):
        super().__init__('risk_judge')
        self.safe_distance = 0.50
        self.escalate_sec = 2.5

        self.subscription = self.create_subscription(
            Float32, '/distance', self.distance_callback, 10)
        self.create_subscription(Float32, '/cfg/safe_distance', self.cfg_distance_cb, 10)
        self.create_subscription(Float32, '/cfg/escalate_ms', self.cfg_escalate_cb, 10)
        self.publisher_ = self.create_publisher(String, '/risk_state', 10)

        self.state = 'NORMAL'
        self.warning_start_time = None

    def cfg_distance_cb(self, msg):
        self.safe_distance = msg.data

    def cfg_escalate_cb(self, msg):
        self.escalate_sec = msg.data / 1000.0

    def distance_callback(self, msg):
        distance = msg.data
        now = time.time()

        # -1.0 = 이번 프레임에 아무도 감지되지 않음 -> 무시 (상태 유지 안 하고 그냥 넘어감)
        if distance < 0:
            return

        if distance <= self.safe_distance:
            if self.state == 'NORMAL':
                self.state = 'WARNING'
                self.warning_start_time = now
                self.get_logger().warn(f'WARNING 진입: {distance:.2f}m (기준 {self.safe_distance:.2f}m)')
            elif self.state == 'WARNING':
                elapsed = now - self.warning_start_time
                if elapsed >= self.escalate_sec:
                    self.state = 'EMERGENCY'
                    self.get_logger().error(f'EMERGENCY 진입: {elapsed:.1f}초간 위험거리 유지')
        else:
            if self.state != 'NORMAL':
                self.get_logger().info(f'정상 복귀: {distance:.2f}m')
            self.state = 'NORMAL'
            self.warning_start_time = None

        out = String()
        out.data = self.state
        self.publisher_.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = RiskJudge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
