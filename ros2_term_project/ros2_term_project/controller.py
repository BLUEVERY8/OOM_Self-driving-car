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
            self.car_info_listener_callback,
            10)

        # 카메라, 라이다 정보를 처리하는 노드에 차량 정보를 전달하는 publisher
        self.car_info_publisher = self.create_publisher(
            String,
            'car_info',
            10
        )

        # line_follower 노드로부터 차량에 전달할 속도 정보를 받아오는 subscription
        self.twist_subscription = self.create_subscription(
            Twist,
            'twist_info',
            self.twist_listener_callback,
            10
        )

        # line_follower 노드로부터 차량 상태(직진, 회전) 정보를 받아오는 subscription
        self.drive_issue_subscription = self.create_subscription(
            String,
            'drive_issue',
            self.drive_issue_listener_callback,
            10
        )

        # 정지선 검출 정보를 받아오는 subscription
        self.stop_issue_subscription = self.create_subscription(
            String,
            'stop_issue',
            self.stop_issue_listener_callback,
            10
        )

        # 종료 지점 검출 정보를 받아오는 subscription
        self.end_issue_subscription = self.create_subscription(
            String,
            'end_issue',
            self.end_issue_listener_callback,
            10
        )

        # 장애물 감지 정보를 받아오는 subscription
        self.obstacle_issue_subscription = self.create_subscription(
            String,
            'obstacle_issue',
            self.obstacle_issue_listener_callback,
            10
        )

        # 보행자 감지 정보를 받아오는 subscription
        self.actor_issue_subscription = self.create_subscription(
            String,
            'actor_issue',
            self.actor_issue_listener_callback,
            10
        )

        # 초기화
        self.twist_publisher_ = None

        # 회전 여부
        self.turn = False
        # 장애물 여부
        self.obstacle_found = False
        # 정지 여부
        self.stop = False
        # 정지선 종류 구분
        self.count = 0

    def car_info_listener_callback(self, msg: Target):
        # 지정 차량
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)

        # 차량에 속도 정보를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, '/demo/' + car + '_cmd_demo', 10)


        time.sleep(3)
        # 차량 출발
        twist = Twist()
        for i in range(200):
            twist.linear.x = 6.0
            self.twist_publisher_.publish(twist)
            time.sleep(0.01)

        # 카메라, 라이다 정보를 처리하는 노드들에 차량 정보 전달
        car_info_msg = String()
        car_info_msg.data = car
        self.car_info_publisher.publish(car_info_msg)



    def twist_listener_callback(self, twist: Twist):
        # 차선을 따라 주행하기 위한 속도 정보 전달
        if not self.stop and not self.obstacle_found:
            self.twist_publisher_.publish(twist)

    def drive_issue_listener_callback(self, msg: String):
        if msg.data == '직진':
            self.turn = False
        elif msg.data == '회전':
            self.turn = True

    def stop_issue_listener_callback(self, msg: String):
        # 언덕 정지선
        if msg.data == '정지' and not self.turn and not self.stop and self.count == 1:
            self.stop = True
            self.count += 1

            twist = Twist()
            twist.angular.y = 0.0

            # 차량이 밀리지 않도록
            for i in range(3500):
                twist.linear.x = 1.3
                self.twist_publisher_.publish(twist)
                time.sleep(0.001)

            self.stop = False

            # 차량 출발
            for i in range(2000):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.001)

        # 평지 정지선
        if msg.data == '정지' and not self.turn and not self.stop and self.count == 0:
            self.stop = True
            self.count += 1
            twist = Twist()

            # 3초간 정지
            for i in range(400):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            # 차량 출발
            for i in range(200):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            self.stop = False

    def end_issue_listener_callback(self, msg: String):
        if msg.data == '종료' and not self.turn:
            self.stop = True
            twist = Twist()

            # 정지선 1m 이내 정차
            for i in range(200):
                twist.linear.x = 2.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_publisher_.publish(twist)
            self.get_logger().info('종료')

            # 테스트 종료
            self.destroy_node()
            return
    def obstacle_issue_listener_callback(self, msg: String):
        # 장애물이 범위를 빠져나갔을 때
        if msg.data == '이동 가능' and self.obstacle_found:
            self.obstacle_found = False
            twist = Twist()
            for i in range(200):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)
            return
        if msg.data == '장애물 감지' and not self.obstacle_found:
            self.obstacle_found = True
            twist = Twist()
            for i in range(10):
                twist.linear.x = 0.0
                twist.angular.y = 0.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.1)



    def actor_issue_listener_callback(self, msg: String):
        #보행자가 지나갔을 시
        if self.obstacle_found and msg.data == '이동 가능':
            self.obstacle_found = False
            self.get_logger().info('보행자 통과')
            twist = Twist()
            for i in range(200):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

        if msg.data == '보행자 감지' and not self.turn:
            self.obstacle_found = True
            twist = Twist()
            for i in range(10):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.1)


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