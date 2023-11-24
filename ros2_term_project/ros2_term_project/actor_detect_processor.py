import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .actor_detector import ActorDetector
import cv_bridge
import time


class ActorDetectProcessor(Node):
    def __init__(self, actor_detector: ActorDetector):
        super().__init__('actor_detect_processor')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        self.actor_issue_publisher_ = self.create_publisher(
            String,
            'actor_issue',
            10
        )

        self.actor_image_subscription_ = None
        self.actor_detector = actor_detector
        self.bridge = cv_bridge.CvBridge()
        self.wait = False

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 주행에 필요한 정보를 받는 subscription 생성
        self.actor_image_subscription_ = self.create_subscription(
            Image,
            '/demo/' + car + '_front_camera/image_raw',
            self.actor_image_callback,
            10)

    def actor_image_callback(self, image: Image):
        msg = String()

        # ros image를 opencv image로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 이미지를 기반으로 보행자 검출
        self.actor_detector.process(img)

        if self.actor_detector._delta is not None and self.actor_detector._delta <= 250:
            msg.data = '보행자 감지'
            self.actor_issue_publisher_.publish(msg)
            self.wait = True
        elif self.wait and self.actor_detector._delta is None:
            msg.data = '이동 가능'
            self.actor_issue_publisher_.publish(msg)
            self.wait = False


def main(args=None):
    rclpy.init(args=args)

    actor_detector = ActorDetector()
    actor_detect_processor = ActorDetectProcessor(actor_detector)

    rclpy.spin(actor_detect_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actor_detect_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()