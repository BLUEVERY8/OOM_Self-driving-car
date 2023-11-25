import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BoxController(Node):

    def __init__(self):
        super().__init__('box_controller')

        # 박스에 속도 정보를 전달할 publisher
        self.publisher_ = self.create_publisher(Twist, '/demo/box_cmd_demo', 10)

        # 박스의 위치 정보를 받을 subscription
        self.subscription_ = self.create_subscription(Odometry, '/demo/box_odom_demo', self.listener_callback, 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.direction = 1
        self.msg = Twist()
        # 박스 위치
        self.y = None

    def listener_callback(self, odom: Odometry):
        self.y = odom.pose.pose.position.y

        # 박스가 차선을 왕복할 수 있도
        if self.direction == 1 and self.y > -62:
            self.direction *= -1
        if self.direction == -1 and self.y < -79:
            self.direction *= -1

    def timer_callback(self):
        self.msg.linear.y = 2.0 * self.direction
        self.publisher_.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)

    box_controller = BoxController()

    rclpy.spin(box_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    box_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()