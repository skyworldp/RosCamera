from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明 launch 参数（仅保留 rviz）
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start rviz2 (true/false)'
    )

    # 获取参数值
    rviz_cfg = LaunchConfiguration('rviz')

    # image_process 节点（模型路径已在代码中硬编码）
    image_process_node = Node(
        package='image_process',
        executable='image_process_node',
        name='image_process_node',
        output='screen'
    )

    # rviz 节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_cfg)
    )

    return LaunchDescription([
        rviz_arg,
        image_process_node,
        rviz_node,
    ])
