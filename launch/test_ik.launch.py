import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Path to the kinematics services launch file
    kinematics_launch_file = os.path.join(
        get_package_share_directory('s_pipe_kinematics'),
        'launch',
        'kinematics_services.launch.py'
    )
    
    return LaunchDescription([
        # Include the services launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinematics_launch_file)
        ),
        
        # Inverse Kinematics Test Client
        Node(
            package='s_pipe_kinematics',
            executable='test_ik_client',
            name='test_ik_client',
            output='screen'
        )
    ])