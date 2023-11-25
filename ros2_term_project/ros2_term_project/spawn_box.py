import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import os
from ament_index_python.packages import get_package_share_directory


class BoxSpawn(Node):

    def __init__(self):
        super().__init__('spawn_entity')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.future = None

    def send_request(self):
        self.req.name = "my_box"
        model_file = '/home/ros2/Ros2Projects/oom_ws/src/ros2_term_project/models/box/model.sdf'

        model_xml = open(model_file).read()
        self.req.xml = model_xml
        self.req.initial_pose.position.x = 0.0
        self.req.initial_pose.position.y = -12.0
        self.req.initial_pose.position.z = 0.2
        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        self.req.initial_pose.orientation.x = quaternion[0]
        self.req.initial_pose.orientation.y = quaternion[1]
        self.req.initial_pose.orientation.z = quaternion[2]
        self.req.initial_pose.orientation.w = quaternion[3]
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = BoxSpawn()
    client.send_request()
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print('response status = ', response.status_message)
            except Exception as e:
                client.get_logger().info('Service call failed %s' % e)
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
