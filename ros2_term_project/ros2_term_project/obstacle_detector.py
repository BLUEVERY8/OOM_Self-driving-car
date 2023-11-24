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

        self.obstacle_issue_publisher_ = self.create_publisher(
            String,
            'obstacle_issue',
            10
        )

        self.lidar_subscription_ = None
        self.bridge = cv_bridge.CvBridge()
        self.wait = False
        self.wait_time = 0

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 주행에 필요한 정보를 받는 subscription 생성
        self.lidar_subscription_ = self.create_subscription(
            LaserScan,
            '/demo/' + car + '_scan', self.scan_callback,
            10)

    def scan_callback(self, scan: LaserScan):
        msg = String()

        min_distance = min(scan.ranges)
        # self.get_logger().info('min_distance: %f' % min_distance)

        if min_distance <= 8.0:
            msg.data = '장애물 감지'
            self.obstacle_issue_publisher_.publish(msg)
            self.wait = True
        if self.wait and min_distance > 7.5:
            msg.data = '이동 가능'
            self.obstacle_issue_publisher_.publish(msg)
            self.wait = False

        if self.wait and self.wait_time > 40:
            msg.data = '이동 가능'
            self.obstacle_issue_publisher_.publish(msg)
            self.wait = False
            self.wait_time = 0
        if self.wait:
            self.wait_time += 1
        # self.get_logger().info('wait_time: %d' % self.wait_time)

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