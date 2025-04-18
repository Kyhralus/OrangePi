import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener_py')
        # 创建一个订阅者，订阅 /plan 话题，消息类型为 Path，队列大小为 10
        self.get_logger().debug("创建了nav2规划路径订阅方(python)!")
        # 创建订阅方
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.listener_callback,
            10)

def listener_callback(self, msg):
    try:
        # 当接收到消息时，打印消息的基本信息
        self.get_logger().info(f'接收到的路径有 {len(msg.poses)} 个位姿')
        # 打印路径中每个位姿的信息
        for i, pose in enumerate(msg.poses):
            if i % 100 == 0:
                position = pose.pose.position
                orientation = pose.pose.orientation
                # 打印 header 信息
                frame_id = pose.header.frame_id
                stamp = pose.header.stamp
                sec = stamp.sec
                nanosec = stamp.nanosec
                self.get_logger().info(f'Pose {i}: '
                                       f'Header: Frame ID={frame_id}, Stamp=(sec={sec}, nanosec={nanosec}) '
                                       f'Position (x={position.x}, y={position.y}, z={position.z}) '
                                       f'Orientation (x={orientation.x}, y={orientation.y}, '
                                       f'z={orientation.z}, w={orientation.w})')
    except Exception as e:
        self.get_logger().error(f"处理路径消息时发生错误: {e}")

def main():
    try:
        # 初始化 ROS2 客户端
        rclpy.init()
        # 调用spin函数，并传入自定义类对象
        rclpy.spin(PathListener())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生未知错误: {e}")
    finally:
        # 释放资源
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    