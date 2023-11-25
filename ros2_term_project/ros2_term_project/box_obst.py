import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class BoxMove(Node):
    PUB_RATE = 1.0

    def __init__(self):
        super().__init__('move')
        self._publisher = self.create_publisher(Twist, 'box_move/cmd_vel', 1)
        timer_period = BoxMove.PUB_RATE
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.twist = Twist()
        self.twist.linear.x = -1.0
        self.i = 0

    def pub_callback(self):
        self.i += 1
        if self.i == 16:
            self.twist.linear.x = self.twist.linear.x * (-1)
            self.i = 0
        self._publisher.publish(self.twist)


def main():
    rclpy.init()
    client = BoxMove()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('error')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
