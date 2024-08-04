from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('obs_avoidance_sim')
    urdf_file = os.path.join(package_share_directory, 'urdf', 'robot.urdf')
    world_file = os.path.join(package_share_directory, 'worlds', 'obstacle_world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'simple_robot', '-file', urdf_file],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        Node(
            package='obs_avoidance_sim',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            output='screen'
        )
    ])
