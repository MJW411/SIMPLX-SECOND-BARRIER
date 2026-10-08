import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Float32, 'test_distance', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.value = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f}')
        self.value += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
