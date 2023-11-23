import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
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

        # 연관
        # 선 검출 기능
        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.end_line_tracker = end_line_tracker

        self.bridge = cv_bridge.CvBridge()
        # 지정 차량 정보
        self.car = sys.argv[1]
        # 회전 여부
        self.turn = False
        # 장애물 여부
        self.obstacle_found = False

        # 지정된 차량에 맞게 해당 카메라 subscription 생성
        if self.car == 'PR001':
            self.image_subscription_ = self.create_subscription(Image, '/demo/PR001_camera/image_raw',
                                                                self.image_callback, 10)
            self.front_image_subscription_ = self.create_subscription(Image, '/demo/PR001_front_camera/image_raw',
                                                                      self.front_image_callback, 10)
            self.lidar_subscription_ = self.create_subscription(LaserScan, '/demo/PR001_scan', self.scan_callback, 10)
            self.publisher_ = self.create_publisher(Twist, '/demo/PR001_cmd_demo', 10)
        elif self.car == 'PR002':
            self.image_subscription_ = self.create_subscription(Image, '/demo/PR002_camera/image_raw',
                                                                self.image_callback, 10)
            self.front_image_subscription_ = self.create_subscription(Image, '/demo/PR002_front_camera/image_raw',
                                                                      self.front_image_callback, 10)
            self.lidar_subscription_ = self.create_subscription(LaserScan, '/demo/PR002_scan', self.scan_callback, 10)
            self.publisher_ = self.create_publisher(Twist, '/demo/PR002_cmd_demo', 10)

        # 센서로부터 처리한 정보를 보내줄 publisher 생성
        self.twist_publisher_ = self.create_publisher(Twist, 'state_update', 10)
        self.stop_issue_publisher_ = self.create_publisher(String, 'stop_issue_update', 10)
        self.end_issue_publisher_ = self.create_publisher(String, 'end_issue_update', 10)

    def image_callback(self, image: Image):
        if self.obstacle_found: return
        # ros image를 opencv image로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 이미지를 기반으로 차선 검출
        self.line_tracker.process(img)

        # 회전 속도 조절
        msg = Twist()
        msg.angular.z = (-1) * self.line_tracker._delta / 110

        # 회전 인식
        if msg.angular.z > 0.3:
            self.turn = True
        else:
            self.turn = False

        self.twist_publisher_.publish(msg)

    def front_image_callback(self, image: Image):
        if self.obstacle_found: return
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        self.end_line_tracker.process(img)
        msg = String()

        # 정지선, 종료선 구분
        if self.end_line_tracker._delta is not None and not self.turn and self.end_line_tracker._delta < 0.1:
            msg.data = '종료'
            self.get_logger().info('종료')
            self.end_issue_publisher_.publish(msg)
            self.destroy_node()
            return
        self.stop_line_tracker.process(img)
        if self.stop_line_tracker._delta is not None and not self.turn and self.stop_line_tracker._delta < 0.01:
            msg.data = '정지'
            self.stop_issue_publisher_.publish(msg)
        else:
            msg.data = ''
            self.end_issue_publisher_.publish(msg)
            self.stop_issue_publisher_.publish(msg)

    def scan_callback(self, msg: LaserScan):
        twist = Twist()
        min_distance = min(msg.ranges)
        self.get_logger().info('min_distance: %f' % min_distance)
        if min_distance < 8.0:
            self.obstacle_found = True
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
        else:
            self.obstacle_found = False
            twist.linear.x = 6.0
            self.publisher_.publish(twist)


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

