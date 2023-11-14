import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
from custom_interface.msg import Target
import cv_bridge

class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self._image_subscription = None
        self._camera_subscription = self.create_subscription(Target, '/start_car', self.listener_callback, 10)
        self._publisher = self.create_publisher(Twist, 'state_update', 10)

    def listener_callback(self, msg: Target):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        self._image_subscription = self.create_subscription(Image, '/demo/' + car + '_camera/' + car + '_image_raw', self.image_callback,
                                                            10)

    def image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.line_tracker.process(img)
        msg = Twist()
        msg.angular.z = (-1) * self.line_tracker._delta / 450
        self.get_logger().info('angular.z = %f' % msg.angular.z)
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = LineTracker()
    follower = LineFollower(tracker)
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

