#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
import subprocess


def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('comm'), 'param.yaml')
    rviz_dir = os.path.join(get_package_share_directory('comm'), 'goliath_crane.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ip_camera_node = ""
    tf_points = ""
    lidar_protect = ""
    camera_protect = ""
    comm = ""
    ros_version = p.communicate()[0]

    print("ROS VERSION: foxy/galactic/humble")
    driver_node = LifecycleNode(package='lslidar_driver',
                                namespace='base',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir],
                                )
    rviz_node = Node(
        package='rviz2',
        namespace='base',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')
    
    ip_camera_node = Node(
        package='camera',
        namespace='base',
        executable='camera_node',
        name='camera_node',
        parameters=[driver_dir]
    )
    tf_points = Node(
        package='transform',
        namespace='base',
        executable='transform_node',
        name='transform_node',
        parameters= [driver_dir]
    )
    lidar_protect = Node(
        package='lidar_protect',
        namespace='base',
        executable='lidar_protect_node',
        name='lidar_protect_node',
        parameters=[driver_dir]
    )   
    camera_protect = Node(
        package='camera_protect',
        namespace='base',
        executable='camera_protect_node',
        name='camera_protect_node',
        parameters=[driver_dir]
    ) 
    comm = Node(
        package='comm',
        namespace='base',
        executable='comm_node',
        name='comm_node',
        parameters=[driver_dir]
    ) 


    return LaunchDescription([
        driver_node, rviz_node, tf_points, ip_camera_node, lidar_protect, camera_protect, comm
    ])
