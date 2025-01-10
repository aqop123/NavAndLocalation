import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription


    def get_params(name):
        return os.path.join(get_package_share_directory('bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))


    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[get_params('serial_driver')],
        ros_arguments=['--ros-args', ],
    )

    lidar_node = ComposableNode(
        package='selfaim_lidar',
        plugin='auto_aim::CustomPoint2PointCloud',
        name='laser_detector',
        parameters=[get_params('laser_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_lidar_node = TimerAction(
        period=2.0,
        actions=[lidar_node],
    )

    
    launch_description_list = [
        delay_serial_node,
        delay_lidar_node]
    
    
    return LaunchDescription(launch_description_list)
