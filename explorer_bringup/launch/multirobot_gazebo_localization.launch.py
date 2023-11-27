#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    name_list = ['robot1','robot2']
    x_list = ['1.0', '1.0']
    y_list = ['1.0', '2.0']
    z_list = ['0.11', '0.11']
    R_list = ['0.0', '0.0']
    P_list = ['0.0', '0.0']
    Y_list = ['0.0', '0.0']
    
    world_file_name = 'map1.world.xml'
    world = os.path.join(get_package_share_directory('explorer_gazebo'),
                         'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_explorer_bringup = get_package_share_directory('explorer_bringup')

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ))
    
    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ))
    
    for i in range(len(name_list)):
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_explorer_bringup, 'onerobot_localization.launch.py')
            ),
            launch_arguments={'name': name_list[i],
                              'x' : x_list[i],
                              'y' : y_list[i],
                              'z' : z_list[i],
                              'R' : R_list[i],
                              'P' : P_list[i],
                              'Y' : Y_list[i]
                              }.items(),
        ))


    return ld