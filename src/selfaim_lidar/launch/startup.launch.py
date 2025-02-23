import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():

    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription


    lidar_node = Node(
        package='selfaim_lidar',
        executable='selfaim_lidar_node',
        name='laser_detector',
        namespace='',
        output='screen',
    )



    
    return LaunchDescription([lidar_node])