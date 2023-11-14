import rclpy
from rclpy.node import Node

from custom_interface.msg import Target
from geometry_msgs.msg import Twist

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Target,
            'start_car',
            self.listener_callback,
            10)
        self.state_subscription = self.create_subscription(Twist, 'state_update', self.state_listener_callback, 10)
        self.publisher_ = None
        self.timer = None

    def listener_callback(self, msg):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        self.publisher_ = self.create_publisher(Twist, 'demo/' + car + '_cmd_demo', 10)

    def state_listener_callback(self, delta: Twist):
        msg = Twist()
        self.get_logger().info('I heard %f' % delta.angular.z)
        msg.angular.z = delta.angular.z
        if delta.angular.z != 0:
            msg.linear.x = 1.0
        else:
            msg.linear.x = 2.0
        self.publisher_.publish(msg)


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