import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .PR002_line_tracker import PR002LineTracker
from .PR001_line_tracker import PR001LineTracker
from custom_interface.msg import Target
import cv_bridge
import sys

class PR001LineFollower(Node):
    def __init__(self, line_tracker: PR001LineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self._image_subscription = self.create_subscription(Image, '/demo/PR001_camera/image_raw',
                                                            self.image_callback, 10)
        self._publisher = self.create_publisher(Twist, 'state_update', 10)

    def image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.line_tracker.process(img)
        msg = Twist()
        msg.angular.z = (-1) * self.line_tracker._delta / 150
        self.get_logger().info('angular.z = %f' % msg.angular.z)
        self._publisher.publish(msg)

class PR002LineFollower(Node):
    def __init__(self, line_tracker: PR002LineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self._image_subscription = self.create_subscription(Image, '/demo/PR002_camera/image_raw',
                                                            self.image_callback, 10)
        self._publisher = self.create_publisher(Twist, 'state_update', 10)

    def image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.line_tracker.process(img)
        msg = Twist()
        msg.angular.z = ((-1) * self.line_tracker._delta / 150)
        self.get_logger().info('angular.z = %f' % msg.angular.z)
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    car = sys.argv[1]
    if car == 'PR001':
        tracker = PR001LineTracker()
        follower = PR001LineFollower(tracker)
    elif car == 'PR002':
        tracker = PR002LineTracker()
        follower = PR002LineFollower(tracker)
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

