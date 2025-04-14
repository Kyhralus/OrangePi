import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

# ============== 路径设置 ==============
# map
map_file = os.path.join(get_package_share_directory('mybot_navigation2'), 'maps', 'map.yaml')
# urdf模型
default_model_path = os.path.join(get_package_share_directory('mybot_description'), 'urdf', 'mybot_description.urdf')
# 启动AMCL节点
nav2_yaml = os.path.join(get_package_share_directory('mybot_description'), 'config', 'amcl_config.yaml')


# ============== 启动文件 ==============
def generate_launch_description():
#     Node_1 = Node(
#         package='package-name', #节点所在的功能包
#         namespace='package-namespace', #命名空间。如果存在同名节点，这一选项会有用.使节点名称前增加命名空间前缀，命名空间不同使系统允许两个相同节点名和主题名不冲突。如果没有唯一的命名空间，当topic消息相同时就无法区分是哪个节点的。
#         executable='execute-name/script-name.py', #表示要运行的可执行文件名或脚本名字.py
#         parameters=[{'parameter-name': parameter-value}], #参数
#         arguments=['-xxx', xxx,  '-xxx', xxx ], #启动参数
#         output='screen', #用于将话题信息打印到屏幕
#         name='node-name' #表示启动后的节点名，可以没有
#         remappings=[ #重映射,将默认节点属性（如节点名称、主题名称、服务名称等），重映射为其它名称。
#             ('/xxx/xxx-new', '/xxx/xxx-old'),
#         ]
#    ),
#     Node_2 = Node(
#         package="Name_2",
#         executable='',
#         name=''
#    )
# 
# 
#    return LaunchDescription([
# 	Node_1,
#     Node_2
#    ])

    amcl_node = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    # 生命循环节点也同样需要启动
    lifecycle_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['amcl']}]  
    )


    return launch.LaunchDescription([
        amcl_node,
        lifecycle_node
    ])

