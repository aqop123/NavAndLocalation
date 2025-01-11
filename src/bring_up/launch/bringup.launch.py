import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    bringup_dir = get_package_share_directory('bring_up')

    ################### user configure parameters for ros2 start ###################
    xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 20.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
    ################### user configure parameters for ros2 end #####################

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    ####################### Livox_ros_driver2 parameters end #########################

    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('bring_up'),
            'config', 'mid360_ver2.yaml'
        ]),
        {
            'use_imu_as_input': True,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'ivox_nearby_type': 6,   # Options: 0, 6, 18, 26
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio_ver2',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio_ver2'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )
    rviz_act_node = GroupAction(
                actions=[rviz_node],
                condition=IfCondition(LaunchConfiguration('rviz'))
            )
    
    # lidar_node = ComposableNode(
    #     package='selfaim_lidar',
    #     plugin='auto_aim::CustomPoint2PointCloud',
    #     name='laser_detector',
    # )

    lidar_node = Node(
        package='selfaim_lidar',
        executable='selfaim_lidar_node',
        name='laser_detector',
    )
    serial_param = os.path.join(
        get_package_share_directory('rc_serial_driver'), 'config', 'serial_driver.yaml')
    rc_serial_driver_node = Node(
        package='rc_serial_driver',
        executable='rc_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[serial_param],
    )
  
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[rc_serial_driver_node],
    )
    # # Assemble the launch description
    # ld = LaunchDescription([
    #     rviz_arg,
    #     laser_mapping_node,
    #     GroupAction(
    #         actions=[rviz_node],
    #         condition=IfCondition(LaunchConfiguration('rviz'))
    #     ),
    # ])

    ld = LaunchDescription()
    ld.add_action(delay_serial_node)
    ld.add_action(livox_driver2_node)
    ld.add_action(rviz_arg)
    ld.add_action(laser_mapping_node)
    ld.add_action(rviz_act_node)
    ld.add_action(lidar_node)
    return ld
