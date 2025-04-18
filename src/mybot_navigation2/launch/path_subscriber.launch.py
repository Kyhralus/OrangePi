import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        # 创建一个订阅者，订阅 /plan 话题，消息类型为 Path，队列大小为 10
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告

    def listener_callback(self, msg):
        # 当接收到消息时，打印消息的基本信息
        self.get_logger().info(f'Received a path with {len(msg.poses)} poses.')
        # 打印路径中每个位姿的信息
        for i, pose in enumerate(msg.poses):
            position = pose.pose.position
            orientation = pose.pose.orientation
            self.get_logger().info(f'Pose {i}: Position (x={position.x}, y={position.y}, z={position.z}) '
                                   f'Orientation (x={orientation.x}, y={orientation.y}, '
                                   f'z={orientation.z}, w={orientation.w})')



rclpy.init()
path_subscriber = PathSubscriber()
rclpy.spin(path_subscriber)
path_subscriber.destroy_node()
rclpy.shutdown()

