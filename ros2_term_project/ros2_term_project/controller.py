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

    def listener_callback(self, msg):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        self.publisher_ = self.create_publisher(Twist, 'demo/' + car + '_cmd_demo', 10)
        twist = Twist()
        twist.linear.x = 6.0
        self.publisher_.publish(twist)
        time.sleep(3)
        self.state_subscription = self.create_subscription(Twist, 'state_update', self.state_listener_callback, 10)
        self.issue_subscription = self.create_subscription(String, 'issue_update', self.issue_listener_callback, 10)

    def state_listener_callback(self, delta: Twist):
        if not self.stop:
            msg = Twist()
            self.get_logger().info('I heard %f' % delta.angular.z)
            msg.angular.z = delta.angular.z
            if delta.angular.z > 0.01:
                msg.linear.x = 3.0
            else:
                msg.linear.x = 6.0
            self.publisher_.publish(msg)

    def issue_listener_callback(self, issue: String):
        msg = Twist()
        if issue == '정지':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.stop = True
            self.publisher_.publish(msg)
            time.sleep(4)
            self.stop = False
            msg.linear.x = 6.0
            self.publisher_.publish(msg)
        elif issue == '종료':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.stop = True
            self.publisher_.publish(msg)
            self.destroy_node()
            rclpy.shutdown()


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