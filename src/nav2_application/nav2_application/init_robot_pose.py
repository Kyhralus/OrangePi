from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main():
    rclpy.init()
    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # ===================== 设置初始位姿 =====================
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -3.0
    initial_pose.pose.orientation.w = 1.0
    # =======================================================
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    rclpy.spin(navigator)
    rclpy.shutdown()
    # if navigator.waitUntilNav2Active():
    #     navigator.get_logger().info("初始化原始位姿成功！")
    #     # 这里关闭节点，因为已经完成了需要的操作
    #     rclpy.shutdown()
    # else:
    #     navigator.get_logger().error("Nav2 未能成功激活，初始化失败。")
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()