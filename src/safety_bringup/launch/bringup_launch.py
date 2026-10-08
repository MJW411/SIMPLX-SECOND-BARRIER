import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={'align_depth.enable': 'true'}.items()
    )

    return LaunchDescription([
        realsense_launch,
        Node(package='person_detector', executable='person_distance',
             name='person_distance', output='screen'),
        Node(package='safety_manager', executable='risk_judge',
             name='risk_judge', output='screen'),
        Node(package='mcu_interface', executable='stop_command',
             name='mcu_interface', output='screen'),
        Node(package='helmet_detector', executable='helmet_check',
             name='helmet_detector', output='screen'),
    ])
