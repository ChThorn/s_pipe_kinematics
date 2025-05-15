import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Forward Kinematics Service Node
        Node(
            package='s_pipe_kinematics',
            executable='fk_service_node',
            name='fk_service',
            output='screen'
        ),
        
        # Inverse Kinematics Service Node
        Node(
            package='s_pipe_kinematics',
            executable='ik_service_node',
            name='ik_service',
            output='screen',
            parameters=[{
                'lambda': 0.1,
                'epsilon': 0.000001,
                'max_iterations': 1000
            }]
        )
    ])