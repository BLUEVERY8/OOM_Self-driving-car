import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .line_tracker import LineTracker
from .end_line_tracker import EndLineTracker
from .stop_line_tracker import StopLineTracker
from custom_interface.msg import Target
import cv_bridge
import sys

class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, stop_line_tracker: StopLineTracker, end_line_tracker: EndLineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.end_line_tracker = end_line_tracker
        self.bridge = cv_bridge.CvBridge()
        self.car = sys.argv[1]
        if self.car == 'PR001':
            self._image_subscription = self.create_subscription(Image, '/demo/PR001_camera/image_raw',
                                                            self.image_callback, 10)
            self._front_image_subscription = self.create_subscription(Image, '/demo/PR001_front_camera/image_raw',
                                                                self.front_image_callback, 10)
        elif self.car == 'PR002':
            self._image_subscription = self.create_subscription(Image, '/demo/PR002_camera/image_raw',
                                                                self.image_callback, 10)
            self._front_image_subscription = self.create_subscription(Image, '/demo/PR002_front_camera/image_raw',
                                                                      self.front_image_callback, 10)
        self._publisher = self.create_publisher(Twist, 'state_update', 10)
        self.stop_issue_publisher = self.create_publisher(String, 'stop_issue_update', 10)
        self.end_issue_publisher = self.create_publisher(String, 'end_issue_update', 10)

    def image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.line_tracker.process(img)
        msg = Twist()
        msg.angular.z = (-1) * self.line_tracker._delta / 150
        self.get_logger().info('angular.z = %f' % msg.angular.z)
        self._publisher.publish(msg)

    def front_image_callback(self, image: Image):
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.stop_line_tracker.process(img)
        self.end_line_tracker.process(img)
        msg = String()
        if self.end_line_tracker._delta is not None and self.end_line_tracker._delta < 3:
            msg.data = '종료'
            self.get_logger().info('종료')
            self.end_issue_publisher.publish(msg)
            self.destroy_node()
            return
        if self.stop_line_tracker._delta is not None and self.stop_line_tracker._delta < 0.005:
            msg.data = '정지'
            self.get_logger().info('정지')
            self.stop_issue_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tracker = LineTracker()
    end_tracker = EndLineTracker()
    stop_tracker = StopLineTracker()
    follower = LineFollower(tracker, stop_tracker, end_tracker)

    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

