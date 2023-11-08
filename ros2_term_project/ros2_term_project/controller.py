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
        self.publisher_ = None
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)
        self.publisher_ = self.create_publisher(Twist, 'demo/' + car + '_cmd_demo', 10)

    def timer_callback(self):
        msg = Twist()
        linear = 2.0
        angular = 0.0
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: linear: "%s"' % linear)


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