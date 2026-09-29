from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'
            ])
        )
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'urdf', 'lebot.urdf.xacro'
    ])
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    twist_mux_params = PathJoinSubstitution([
        FindPackageShare('twist_mux'), 'config', 'twist_mux_topics.yaml'# ??????
    ])

    return LaunchDescription([
        lidar_launch,
        Node(
            package='robot_state_publisher',
            namespace='description',
            executable='robot_state_publisher',
            name='state_pub',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='odometry',
            namespace='odom',
            executable='odometry',
            name='odom',
        ),
        Node(
            package='twist_mux',
            namespace='mux',
            executable='twist_mux',
            name='mux',
            parameters=[twist_mux_params],
        ),
        Node(
            package='controller',
            namespace='controller',
            executable='controller',
            name='gamepad',
            remappings=[('cmd_vel', 'joy_vel')],
        ),
    ])