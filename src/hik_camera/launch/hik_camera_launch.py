import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch arguments (可从命令行覆盖)
    declare_exposure = DeclareLaunchArgument("exposure_time", default_value="2000.0",
                                            description="Exposure time (float)")
    declare_gain = DeclareLaunchArgument("gain", default_value="7.0", description="Gain (float)")
    declare_trigger = DeclareLaunchArgument("trigger_mode", default_value="false",
                                            description="Trigger mode (bool)")
    declare_frame_rate = DeclareLaunchArgument("frame_rate", default_value="200.0",
                                               description="Frame rate (Hz) for SDK")
    # 默认使用 SDK 的 BGR8 常量（如果需要可用 decimal 值覆盖）
    declare_pixel_format = DeclareLaunchArgument("pixel_format", default_value=str(35127317),
                                                 description="Pixel format (SDK integer)")
    declare_frame_id = DeclareLaunchArgument("frame_id", default_value="hik_camera_optical_frame",
                                             description="tf frame_id for published images")
    # RViz optional arguments
    declare_rviz = DeclareLaunchArgument("rviz", default_value="true", description="Launch rviz2 (true/false)")
    declare_rviz_config = DeclareLaunchArgument("rviz_config", default_value="", description="Path to RViz2 config file")

    # Node 参数（使用 LaunchConfiguration，使得命令行 -p/--ros-args 可覆盖）
    params = {
        "exposure_time": LaunchConfiguration("exposure_time"),
        "gain": LaunchConfiguration("gain"),
        "trigger_mode": LaunchConfiguration("trigger_mode"),
        "frame_rate": LaunchConfiguration("frame_rate"),
        "pixel_format": LaunchConfiguration("pixel_format"),
        "frame_id": LaunchConfiguration("frame_id"),
    }

    hik_node = Node(
        package="hik_camera",
        executable="hik_camera_node",
        name="hik_camera_node",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            declare_exposure,
            declare_gain,
            declare_trigger,
            declare_frame_rate,
            declare_pixel_format,
            declare_frame_id,
                declare_rviz,
                declare_rviz_config,
            hik_node,
                # RViz node (conditionally launched)
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", LaunchConfiguration("rviz_config")],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                ),
        ]
    )
