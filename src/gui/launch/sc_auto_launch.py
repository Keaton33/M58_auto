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
    driver_dir = os.path.join(get_package_share_directory('gui'), 'param.yaml')
    print(driver_dir)
    rviz_dir = os.path.join(get_package_share_directory('gui'), 'sc_auto.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    
    driver_node = ""
    transform_node = ""
    camera_node = ""
    comm_node = ""
    gui_node = ""
    hoist_ctrl_node = ""
    marker_node = ""
    spss_node = ""
    trolley_ctrl_node = ""
    rviz_node = ""


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
    transform_node = Node(
        package='transform',
        namespace='base',
        executable='transform_node',
        name='transform_node',
        parameters=[driver_dir]
    )
    rviz_node = Node(
        package='rviz2',
        namespace='base',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen'
    )
    camera_node = Node(
        package='camera',
        namespace='base',
        executable='camera_node',
        name='camera_node',
        parameters=[driver_dir]
    )
    comm_node = Node(
        package='comm',
        namespace='base',
        executable='comm_node',
        name='comm_node',
        parameters= [driver_dir]
    )
    gui_node = Node(
        package='gui',
        namespace='base',
        executable='gui_node',
        name='gui_node',
        parameters=[driver_dir]
    )   
    hoist_ctrl_node = Node(
        package='hoist_ctrl',
        namespace='base',
        executable='hoist_ctrl_node',
        name='hoist_ctrl_node',
        parameters=[driver_dir]
    ) 
    marker_node = Node(
        package='marker',
        namespace='base',
        executable='marker_node',
        name='marker_node',
        parameters=[driver_dir]
    ) 
    spss_node = Node(
        package='spss',
        namespace='base',
        executable='spss_node',
        name='spss_node',
        parameters=[driver_dir]
    ) 
    trolley_ctrl_node = Node(
        package='trolley_ctrl',
        namespace='base',
        executable='trolley_ctrl_node',
        name='trolley_ctrl_node',
        parameters=[driver_dir]
    )

    return LaunchDescription([
        driver_node,
        transform_node,
        camera_node,
        comm_node,
        gui_node,
        hoist_ctrl_node,
        marker_node,
        spss_node,
        trolley_ctrl_node,
        # rviz_node,
    ])
