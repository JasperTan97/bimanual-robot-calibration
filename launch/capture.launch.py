"""Launch the cone_calib capture node for one robot.

Example (service-driven):
    ros2 launch cone_calib capture.launch.py robot_id:=ur3e

For the interactive terminal UX (Enter/n/u/s/q) run the node directly so stdin
is attached:
    ros2 run cone_calib capture_node --ros-args \
        -p config_file:=<share>/config/ur3e.yaml \
        -p plate_config:=<share>/config/plate.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for a single-robot capture session."""
    robot_id = LaunchConfiguration('robot_id')
    pkg = FindPackageShare('cone_calib')
    config_file = PathJoinSubstitution(
        [pkg, 'config', [robot_id, TextSubstitution(text='.yaml')]])
    plate = PathJoinSubstitution([pkg, 'config', 'plate.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', description='robot config to load: fr3 | ur3e'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='session output dir (default <cwd>/data/cone_calib)'),
        DeclareLaunchArgument('interactive', default_value='false',
                              description='true only if stdin is attached (prefer ros2 run)'),
        Node(
            package='cone_calib',
            executable='capture_node',
            name='cone_calib_capture',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'config_file': config_file,
                'plate_config': plate,
                'output_dir': LaunchConfiguration('output_dir'),
                'interactive': LaunchConfiguration('interactive'),
            }],
        ),
    ])
