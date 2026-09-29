# bullseye
# bullseye


import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generte_launch_desceiption():

    lidar_share=FindPackageShare('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(PathJoinSubstitution([lidar_share, 'launch', 'sllidar_c1_launch.py']))
    urdf_share=FindPackageShare('robot_description')
    urdf_file=os.path.join(urdf_share,'urdf','lebot.urdf.xacro')

    return LaunchDescription([
        lidar_launch,
        Node(
            package='robot_state_publisher',
            namespace='description',
            executable='robot_state_publisher',
            name='state_pub',
            arguments=['--ros-args', '-p', 'robot_description:=$(xacro /home/jacintus/Projects/robotAIWeeek-issa/src/robot_description/urdf/lebot.urdf.xacro)']
        ),
        Node(
            package='odometry',
            namespace='odom',
            executable='odometry',
            name='odom',
            arguments=[]
        ),
        Node(
            package='twist_mux',
            namespace='mux',
            executable='twist_mux',
            name='mux',
            arguments=['--ros-args', '--params-file', '~/Projects/robotAIWeeek-issa/src/external/twist_mux/config/twist_mux_topics.yaml']
        ),
        Node(
            package='controller',
            namespace='controller',
            executable='controller',
            name='gamepad',
            arguments=['--ros-args', '-r', 'cmd_vel:=joy_vel']
        )
    ])