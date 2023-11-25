import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .end_line_tracker import EndLineTracker
import cv_bridge

class EndLineDetector(Node):
    def __init__(self, end_line_tracker: EndLineTracker):
        super().__init__('end_line_detector')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 종료 지점 감지 정보를 전달할 publisher
        self.end_issue_publisher_ = self.create_publisher(
            String,
            'end_issue',
            10
        )

        self.end_line_image_subscription_ = None
        self.end_line_tracker = end_line_tracker
        self.bridge = cv_bridge.CvBridge()

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 전방 카메라 정보를 받는 subscription 생성
        self.end_line_image_subscription_ = self.create_subscription(
            Image,
            '/demo/' + car + '_front_camera/image_raw',
            self.end_line_image_callback,
            10)

    def end_line_image_callback(self, image: Image):
        # ros image를 opencv image로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 이미지를 기반으로 종료선 검출
        self.end_line_tracker.process(img)

        #종료선 확인
        if self.end_line_tracker._delta is not None and self.end_line_tracker._delta < 30:
            msg = String()

            msg.data = '종료'
            self.end_issue_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    end_line_tracker = EndLineTracker()
    end_line_detector = EndLineDetector(end_line_tracker)

    rclpy.spin(end_line_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    end_line_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()