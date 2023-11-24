import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge
class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')

        # 연관
        # 선 검출 기능
        self.line_tracker = line_tracker
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )
        self.twist_info_publisher_ = self.create_publisher(
            Twist,
            'twist_info',
            10
        )
        self.drive_issue_publisher_ = self.create_publisher(
            String,
            'drive_issue',
            10
        )
        self.image_subscription_ = None
        self.bridge = cv_bridge.CvBridge()

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 주행에 필요한 정보를 받는 subscription 생성
        self.image_subscription_ = self.create_subscription(Image, '/demo/' + car + '_camera/image_raw',
                                                            self.lane_image_callback, 10)

    def lane_image_callback(self, image: Image):
        # ros image를 opencv image로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 이미지를 기반으로 차선 검출
        self.line_tracker.process(img)

        # 회전 속도 조절
        twist = Twist()
        msg = String()
        twist.angular.z = (-1) * self.line_tracker._delta / 110
        # self.get_logger().info('angular.z: %f' % twist.angular.z)

        # 방향 조정 최대치 설정
        if twist.angular.z > 0.7:
            twist.angular.z = 0.7

        # 회전 인식
        if twist.angular.z > 0.1:
            msg.data = '회전'
        else:
            msg.data = '직진'

        # 회전 시 감속
        if twist.angular.z > 0.3:
            twist.linear.x = 3.0
        else:
            twist.linear.x = 6.0

        self.drive_issue_publisher_.publish(msg)
        self.twist_info_publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    tracker = LineTracker()

    follower = LineFollower(tracker)

    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

