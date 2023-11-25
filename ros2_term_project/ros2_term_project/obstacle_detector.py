import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv_bridge
import time


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 장애물 감지 정보를 전달하는 publisher
        self.obstacle_issue_publisher_ = self.create_publisher(
            String,
            'obstacle_issue',
            10
        )

        self.lidar_subscription_ = None
        self.bridge = cv_bridge.CvBridge()
        # 장애물 감지
        self.wait = False
        self.wait_time = 0

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 라이다 정보를 받는 subscription 생성
        self.lidar_subscription_ = self.create_subscription(
            LaserScan,
            '/demo/' + car + '_scan', self.scan_callback,
            10)

    def scan_callback(self, scan: LaserScan):
        msg = String()

        min_distance = min(scan.ranges)
        if min_distance < 12:
            self.get_logger().info('min_distance: %f' % min_distance)

        # 주행 범위에 장애물이 감지되면
        if min_distance <= 8.5:
            msg.data = '장애물 감지'
            self.obstacle_issue_publisher_.publish(msg)
            self.wait = True
            time.sleep(1)
            return

        if not self.wait:
            msg.data = ''
            self.obstacle_issue_publisher_.publish(msg)
            self.wait = False

        # 장애물이 지나갔을 시
        if self.wait and min_distance > 7.5:
            for i in range(20):
                msg.data = '이동 가능'
                self.obstacle_issue_publisher_.publish(msg)
                time.sleep(0.1)
            self.wait = False
            msg.data = ''
            self.obstacle_issue_publisher_.publish(msg)
            self.destroy_node()
            return


def main(args=None):
    rclpy.init(args=args)

    obstacle_detector = ObstacleDetector()

    rclpy.spin(obstacle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()