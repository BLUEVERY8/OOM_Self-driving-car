import rclpy
from rclpy.node import Node
import time
from custom_interface.msg import Target
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            Target,
            'start_car',
            self.listener_callback,
            10)
        self.state_subscription_ = None
        self.issue_subscription_ = None
        self.twist_publisher_ = None
        self.stop = False
        self.count = 0  # 정지선 종류 구분
    def listener_callback(self, msg):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        # 차량에 선속도, 각속도를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, 'demo/' + car + '_cmd_demo', 10)

        # 차량 출발
        twist = Twist()
        for i in range(2):
            twist.linear.x = 6.0
            self.twist_publisher_.publish(twist)
            time.sleep(1)
        # 차량이 지정되면 주행에 필요한 정보를 받는 subscription 생성
        self.state_subscription_ = self.create_subscription(Twist, 'state_update', self.state_listener_callback, 10)
        self.stop_subscription_ = self.create_subscription(String, 'stop_issue_update', self.stop_listener_callback, 10)
        self.end_subscription_ = self.create_subscription(String, 'end_issue_update', self.end_listener_callback, 10)
    def state_listener_callback(self, delta: Twist):
        # 차량이 정지해있지 않을 경우
        if not self.stop:
            msg = Twist()
            # 방향 조정 최대치 설정
            if delta.angular.z > 1.0:
                delta.angular.z = 1.0
            msg.angular.z = delta.angular.z
            # 회전 시 감속
            if delta.angular.z > 0.3:
                msg.linear.x = 3.0
            else:
                msg.linear.x = 6.0
            self.twist_publisher_.publish(msg)
            self.get_logger().info('선속도: %f' % msg.linear.x)
    def stop_listener_callback(self, issue: String):
        msg = Twist()
        # 정지선 감지
        if not self.stop and issue.data == '정지' and self.count < 2:
            self.stop = True
            self.count += 1
            self.get_logger().info('정지')
            # 언덕 정지선일 경우
            if self.count == 2:
                for i in range(400):
                    msg.linear.x = 1.0

                    self.twist_publisher_.publish(msg)
                    time.sleep(0.01)
            else:
                for i in range(400):
                    msg.linear.x = 0.0

                    self.twist_publisher_.publish(msg)
                    time.sleep(0.01)

            self.stop = False
            for i in range(30):
                msg.linear.x = 6.0
                self.twist_publisher_.publish(msg)
                time.sleep(0.1)

    def end_listener_callback(self, issue: String):
        msg = Twist()
        # 종료선 감지
        if issue.data == '종료':
            self.stop = True

            # 정지선 1m 이내 정차
            for i in range(20):
                msg.linear.x = 1.3
                self.twist_publisher_.publish(msg)
                time.sleep(0.1)
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.twist_publisher_.publish(msg)

            # 주행 종료
            self.destroy_node()
            return
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()