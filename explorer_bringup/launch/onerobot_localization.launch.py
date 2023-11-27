#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def one_robot_spawn_and_localization(context, *args, **kwargs):
    name = LaunchConfiguration('name', default='robot').perform(context)
    x = LaunchConfiguration('x', default='1.0').perform(context)
    y = LaunchConfiguration('y', default='1.0').perform(context)
    z = LaunchConfiguration('z', default='0.11').perform(context)
    R = LaunchConfiguration('R', default='0.0').perform(context)
    P = LaunchConfiguration('P', default='0.0').perform(context)
    Y = LaunchConfiguration('Y', default='0.0').perform(context)

    xacro_name = 'turtlebot3_' + TURTLEBOT3_MODEL
    xacro_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models/xacro',
        xacro_name + '.sdf.xacro'
    )
    xacro_doc = xacro.process_file(xacro_path, mappings={'robot_name': name})
    sdf_doc = xacro_doc.toprettyxml(indent='  ')
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models/xacro',
        'tmp_'+name+'.sdf')
    with open(sdf_path, 'w') as file:
        file.truncate()
        file.write(sdf_doc)
    file.close()

    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  get_package_share_directory('explorer_cartographer'), 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default=name+'_lds_2d_loc_only.lua')

    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', name,  # 替换为你的机器人名称
                '-file', sdf_path,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', R,
                '-P', P,
                '-Y', Y,
                '-robot_namespace', name,
                ],
            ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace = name, #Topic 前缀
            output='screen',
            parameters=[{'frame_prefix': name+'/', #tf frame 前缀
                        'use_sim_time': True,
                        'robot_description': robot_desc}]
            ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace = name,
            output='screen',
            parameters=[{
                         'use_sim_time': True}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                ],
            ),]

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='robot',
            description='This the name space of the robot'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='1.0',
            description='This the initial x value of the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='1.0',
            description='This the initial y value of the robot'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.11',
            description='This the initial z value of the robot'
        ),
        DeclareLaunchArgument(
            'R',
            default_value='0.0',
            description='This the initial R value of the robot'
        ),
        DeclareLaunchArgument(
            'P',
            default_value='0.0',
            description='This the initial P value of the robot'
        ),
        DeclareLaunchArgument(
            'Y',
            default_value='0.0',
            description='This the initial Y value of the robot'
        ),
        OpaqueFunction(function=one_robot_spawn_and_localization)
    ])