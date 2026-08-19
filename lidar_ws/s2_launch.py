"""Launch the RPLIDAR S2 driver.

Every parameter is exposed as a launch argument so the common cases can be
handled from the command line without editing the YAML:

    ros2 launch rplidar_s2_driver s2_launch.py
    ros2 launch rplidar_s2_driver s2_launch.py serial_port:=/dev/ttyUSB1
    ros2 launch rplidar_s2_driver s2_launch.py use_static_tf:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("serial_baudrate", default_value="1000000"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("topic", default_value="scan"),
        DeclareLaunchArgument("range_max", default_value="30.0"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("angle_compensate", default_value="true"),
        DeclareLaunchArgument("samples_per_scan", default_value="1600"),
        DeclareLaunchArgument("min_quality", default_value="0"),
        # Convenience for bench testing without a robot description: RViz
        # needs a TF entry for frame_id, and with no URDF loaded nothing
        # provides one. Not for use on the real robot -- there the URDF
        # defines the laser's true mounting pose.
        DeclareLaunchArgument(
            "use_static_tf", default_value="false",
            description="Publish a base_link->frame_id identity transform"),
    ]

    lidar = Node(
        package="rplidar_s2_driver",
        executable="s2_node",
        name="rplidar_s2",
        output="screen",
        emulate_tty=True,      # keeps log colouring and flushes immediately
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "serial_baudrate": LaunchConfiguration("serial_baudrate"),
            "frame_id": LaunchConfiguration("frame_id"),
            "topic": LaunchConfiguration("topic"),
            "range_max": LaunchConfiguration("range_max"),
            "inverted": LaunchConfiguration("inverted"),
            "angle_compensate": LaunchConfiguration("angle_compensate"),
            "samples_per_scan": LaunchConfiguration("samples_per_scan"),
            "min_quality": LaunchConfiguration("min_quality"),
        }],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_tf",
        arguments=["0", "0", "0", "0", "0", "0",
                   "base_link", LaunchConfiguration("frame_id")],
        condition=IfCondition(LaunchConfiguration("use_static_tf")),
    )

    return LaunchDescription(args + [lidar, static_tf])
