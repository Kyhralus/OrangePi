import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener_py')
        self.get_logger().debug("创建了nav2规划路径订阅方(python)!")
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            self.get_logger().info(f'接收到的路径有 {len(msg.poses)} 个位姿')
            for i, pose in enumerate(msg.poses):
                if i % 10 == 0:
                    self.print_pose_info(i, pose)
        except Exception as e:
            self.get_logger().error(f"处理路径消息时发生错误: {e}")

    def print_pose_info(self, index, pose):
        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(f'Pose {index}: '
                               f'Position (x={position.x}, y={position.y}, z={position.z}) '
                               f'Orientation (x={orientation.x}, y={orientation.y}, '
                               f'z={orientation.z}, w={orientation.w})')


def main():
    try:
        rclpy.init()
        rclpy.spin(PathListener())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生未知错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    