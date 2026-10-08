import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

class DistanceReader(Node):
    def __init__(self):
        super().__init__('distance_reader')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/distance', 10)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        cx, cy = w // 2, h // 2
        region = depth_image[cy-5:cy+5, cx-5:cx+5]
        valid = region[region > 0]
        if len(valid) == 0:
            return
        distance_m = float(np.median(valid)) / 1000.0

        out = Float32()
        out.data = distance_m
        self.publisher_.publish(out)
        self.get_logger().info(f'Distance: {distance_m:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = DistanceReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
