import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog


class Nav2BehaviorsSubscriber(Node):
    def __init__(self):
        super().__init__('nav2_behaviors_subscriber')
        # 创建订阅者，订阅 /behavior_tree_log 话题，消息类型为 BehaviorTreeLog，队列大小为 10
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告

    def listener_callback(self, msg):
        # 打印接收到的行为树日志消息
        self.get_logger().info(f"Received BehaviorTreeLog message:")
        self.get_logger().info(f"  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.get_logger().info(f"  Node name: {msg.node_name}")
        self.get_logger().info(f"  Status: {msg.status}")
        for event in msg.events:
            self.get_logger().info(f"  Event: {event}")


def main(args=None):
    rclpy.init(args=args)
    nav2_behaviors_subscriber = Nav2BehaviorsSubscriber()
    rclpy.spin(nav2_behaviors_subscriber)
    nav2_behaviors_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    