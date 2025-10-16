from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start rviz2 (true/false)')
    rviz_cfg = LaunchConfiguration('rviz')

    hik_node = Node(
        package='hik_video_publisher_cpp',
        executable='hik_video_publisher',
        name='hik_video_publisher',
        output='screen',
        # parameters are present for compatibility but node uses hardcoded values
        parameters=[
            {'video_path': 'video.mp4'},
            {'publish_fps': 30.0}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_cfg)
    )

    return LaunchDescription([
        rviz_arg,
        hik_node,
        rviz_node,
    ])
