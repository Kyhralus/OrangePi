from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from nav2_msgs.action import NavigateToPose
import time
from nav_msgs.msg import Path  # 导入路径消息类型

# 全局变量，用于存储规划的路径
planned_path = None

# 路径信息回调函数
def path_callback(msg):
    global planned_path
    try:
        planned_path = msg
        print("Received a new path with", len(msg.poses), "poses.")
        # 打印header信息
        print(f"Path header: frame_id = {msg.header.frame_id}, stamp = {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        for i, pose_stamped in enumerate(msg.poses):
            pose = pose_stamped.pose
            print(f"Pose {i}:")
            print(f"  Position: x = {pose.position.x}, y = {pose.position.y}, z = {pose.position.z}")
            print(f"  Orientation: x = {pose.orientation.x}, y = {pose.orientation.y}, z = {pose.orientation.z}, w = {pose.orientation.w}")
    except Exception as e:
        print(f"Error processing path message: {e}")


rclpy.init()
navigator = BasicNavigator()

# 创建路径信息订阅者
node = rclpy.create_node('navigation_node')
path_subscription = node.create_subscription(Path, '/plan', path_callback, 10)

# 设置初始位姿
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.orientation.w = 1.0
navigator.setInitialPose(initial_pose)

# 等待Nav2系统准备好
navigator.waitUntilNav2Active()

# 设置导航目标
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

# 开始导航
navigator.goToPose(goal_pose)

last_print_time = time.time()
# 循环检查导航状态
while not navigator.isTaskComplete():
    rclpy.spin_once(node)  # 处理路径数据
    current_time = time.time()
    if current_time - last_print_time > 1:
        current_pose = navigator.getFeedback().current_pose.pose
        print(f"Current position: x = {current_pose.position.x}, y = {current_pose.position.y}")
        last_print_time = current_time

# 获取导航结果
result = navigator.getResult()
if result == NavigateToPose.Result.SUCCEEDED:
    print('Goal succeeded!')
elif result == NavigateToPose.Result.CANCELED:
    print('Goal was canceled!')
elif result == NavigateToPose.Result.FAILED:
    print('Goal failed!')

rclpy.shutdown()
