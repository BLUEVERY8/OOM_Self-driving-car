import rclpy
from rclpy.node import Node
import time
from custom_interface.msg import Target
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Target,
            'start_car',
            self.listener_callback,
            10)
        self.state_subscription = None
        self.issue_subscription = None
        self.publisher_ = None
        self.stop = False
        self.turn = False

    def listener_callback(self, msg):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        self.publisher_ = self.create_publisher(Twist, 'demo/' + car + '_cmd_demo', 10)
        twist = Twist()
        twist.linear.x = 6.0
        self.publisher_.publish(twist)
        time.sleep(3)
        self.state_subscription = self.create_subscription(Twist, 'state_update', self.state_listener_callback, 10)
        self.stop_subscription = self.create_subscription(String, 'stop_issue_update', self.stop_listener_callback, 10)
        self.end_subscription = self.create_subscription(String, 'end_issue_update', self.end_listener_callback, 10)

    def state_listener_callback(self, delta: Twist):
        if not self.stop:
            msg = Twist()
            self.get_logger().info('I heard %f' % delta.angular.z)
            msg.angular.z = delta.angular.z
            if delta.angular.z > 0.02:
                self.turn = True
                msg.linear.x = 3.0
            else:
                self.turn = False
                msg.linear.x = 6.0
            self.publisher_.publish(msg)

    def stop_listener_callback(self, issue: String):
        msg = Twist()

        if not self.turn and issue.data == '정지':
            self.stop = True
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.publisher_.publish(msg)
            time.sleep(3)
            self.stop = False

    def end_listener_callback(self, issue: String):
        msg = Twist()

        if issue.data == '종료':
            self.stop = True
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.publisher_.publish(msg)
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