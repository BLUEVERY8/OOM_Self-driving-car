import rclpy
from rclpy.node import Node
from custom_interface.msg import Target
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            Target,
            'start_car',
            self.listener_callback,
            10)

        self.car_info_publisher = self.create_publisher(
            String,
            'car_info',
            10
        )

        self.twist_subscription = self.create_subscription(
            Twist,
            'twist_info',
            self.twist_listener_callback,
            10
        )

        self.drive_issue_subscription = self.create_subscription(
            String,
            'drive_issue',
            self.drive_issue_listener_callback,
            10
        )

        self.stop_issue_subscription = self.create_subscription(
            String,
            'stop_issue',
            self.stop_issue_listener_callback,
            10
        )

        self.end_issue_subscription = self.create_subscription(
            String,
            'end_issue',
            self.end_issue_listener_callback,
            10
        )

        self.obstacle_issue_subscription = self.create_subscription(
            String,
            'obstacle_issue',
            self.obstacle_issue_listener_callback,
            10
        )

        self.actor_issue_subscription = self.create_subscription(
            String,
            'actor_issue',
            self.actor_issue_listener_callback,
            10
        )

        self.twist_publisher_ = None

        # 회전 여부
        self.turn = False
        # 장애물 여부
        self.obstacle_found = False
        # 정지 여부
        self.stop = False
        self.count = 0

    def listener_callback(self, msg: Target):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)

        # 차량에 선속도, 각속도를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, '/demo/' + car + '_cmd_demo', 10)

        # 차량 출발
        twist = Twist()
        for i in range(10):
            twist.linear.x = 6.0
            self.twist_publisher_.publish(twist)
            self.get_logger().info('선 속도: "%f"' % twist.linear.x)
            time.sleep(0.1)

        car_info_msg = String()
        car_info_msg.data = car
        self.car_info_publisher.publish(car_info_msg)

    def twist_listener_callback(self, twist: Twist):
        if not self.stop and not self.obstacle_found:
            self.twist_publisher_.publish(twist)
            # self.get_logger().info('각 속도: %f' % twist.angular.z)
            # self.get_logger().info('선 속도: %f' % twist.linear.x)

    def drive_issue_listener_callback(self, msg: String):
        if msg.data == '직진':
            self.turn = False
        elif msg.data == '회전':
            self.turn = True

    def stop_issue_listener_callback(self, msg: String):
        if msg.data == '정지' and not self.turn and not self.stop and self.count == 0:
            self.stop = True
            self.count += 1
            self.get_logger().info('count: %d' % self.count)
            twist = Twist()

            for i in range(400):
                twist.linear.x = 0.0

                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.01)

            for i in range(10):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.1)

            self.stop = False
        elif msg.data == '정지' and not self.turn and not self.stop and self.count == 1:
            self.stop = True
            self.count += 1
            self.get_logger().info('count: %d' % self.count)
            twist = Twist()

            for i in range(100):
                twist.linear.x = 3.0

                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.01)

            for i in range(400):
                twist.linear.x = 1.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.01)

            self.stop = False



    def end_issue_listener_callback(self, msg: String):
        if msg.data == '종료' and not self.turn:
            self.stop = True
            twist = Twist()

            # 정지선 1m 이내 정차
            for i in range(25):
                twist.linear.x = 2.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.1)

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_publisher_.publish(twist)
            self.get_logger().info('각 속도: %f' % twist.angular.z)
            self.get_logger().info('선 속도: %f' % twist.linear.x)
            self.get_logger().info('종료')

            self.destroy_node()
            return
    def obstacle_issue_listener_callback(self, msg: String):
        if msg.data == '장애물 감지' and not self.obstacle_found:
            self.obstacle_found = True
            twist = Twist()
            for i in range(100):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.01)
        elif msg.data == '이동 가능' and self.obstacle_found:
            self.obstacle_found = False
            twist = Twist()
            for i in range(150):
                twist.linear.x = 5.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.01)
    def actor_issue_listener_callback(self, msg: String):
        if msg.data == '보행자 감지':
            self.obstacle_found = True
            twist = Twist()
            for i in range(10):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.1)
        if self.obstacle_found and msg.data == '이동 가능':
            self.obstacle_found = False
            twist = Twist()
            for i in range(10):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                self.get_logger().info('선 속도: %f' % twist.linear.x)
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)
    # controller.get_logger().info('차선 침범" %d회' % tracker._invasion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()