import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .stop_line_tracker import StopLineTracker
import cv_bridge
import time

class StopLineDetector(Node):
    def __init__(self, stop_line_tracker: StopLineTracker):
        super().__init__('stop_line_detector')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        self.stop_issue_publisher_ = self.create_publisher(
            String,
            'stop_issue',
            10
        )

        self.stop_line_image_subscription_ = None
        self.stop_line_tracker = stop_line_tracker
        self.bridge = cv_bridge.CvBridge()


    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 주행에 필요한 정보를 받는 subscription 생성
        self.stop_line_image_subscription_ = self.create_subscription(
            Image,
            '/demo/' + car + '_front_camera/image_raw',
            self.stop_line_image_callback,
            10)

    def stop_line_image_callback(self, image: Image):
        # ros image를 opencv image로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 이미지를 기반으로 정지선 검출
        self.stop_line_tracker.process(img)

        # 정지선 확인
        if self.stop_line_tracker._delta is not None and self.stop_line_tracker._delta < 3.0:
            msg = String()
            msg.data = '정지'
            self.stop_issue_publisher_.publish(msg)
        else:
            msg = String()
            msg.data = ''
            self.stop_issue_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    stop_line_tracker = StopLineTracker()
    stop_line_detector = StopLineDetector(stop_line_tracker)

    rclpy.spin(stop_line_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stop_line_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()