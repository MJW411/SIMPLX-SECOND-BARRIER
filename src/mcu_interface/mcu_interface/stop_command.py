import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class McuInterface(Node):
    def __init__(self):
        super().__init__('mcu_interface')
        self.subscription = self.create_subscription(
            String, '/risk_state', self.state_callback, 10)
        self.last_state = None

        # TODO: 아두이노 준비되면 여기서 시리얼 포트 열기
        # import serial
        # self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def state_callback(self, msg):
        state = msg.data
        if state == self.last_state:
            return  # 상태 안 바뀌었으면 무시 (매 프레임 스팸 방지)
        self.last_state = state

        if state == 'EMERGENCY':
            self.send_to_mcu('STOP')
        elif state == 'NORMAL':
            self.send_to_mcu('RECOVER')
        else:
            self.get_logger().info(f'상태 변경: {state}')

    def send_to_mcu(self, command):
        # TODO: 아두이노 연결되면 아래 한 줄로 교체
        # self.ser.write((command + '\n').encode())
        self.get_logger().warn(f'[시뮬레이션] MCU로 전송: {command} (실제 시리얼 연결 없음)')

def main(args=None):
    rclpy.init(args=args)
    node = McuInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
