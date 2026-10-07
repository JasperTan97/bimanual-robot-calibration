"""Publish a solved base-to-base transform as a static TF.

The cone_calib solver prints a ready-to-run invocation of this launch file with
the solved numbers filled in (translation + XYZW quaternion). Example:

    ros2 launch cone_calib publish_calibration.launch.py \
        parent:=ur3e_base_link child:=fr3_link0 \
        x:=-0.15 y:=0.80 z:=0.0 qx:=0.0 qy:=0.0 qz:=0.0 qw:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the static_transform_publisher launch description."""
    args = [
        DeclareLaunchArgument('parent', default_value='ur3e_base_link'),
        DeclareLaunchArgument('child', default_value='fr3_link0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('qx', default_value='0.0'),
        DeclareLaunchArgument('qy', default_value='0.0'),
        DeclareLaunchArgument('qz', default_value='0.0'),
        DeclareLaunchArgument('qw', default_value='1.0'),
    ]
    publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cone_calib_static_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--qx', LaunchConfiguration('qx'),
            '--qy', LaunchConfiguration('qy'),
            '--qz', LaunchConfiguration('qz'),
            '--qw', LaunchConfiguration('qw'),
            '--frame-id', LaunchConfiguration('parent'),
            '--child-frame-id', LaunchConfiguration('child'),
        ],
    )
    return LaunchDescription(args + [publisher])
