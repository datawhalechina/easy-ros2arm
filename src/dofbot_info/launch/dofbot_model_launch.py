from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_path, get_package_share_directory

import os

def generate_launch_description():
    urdf_package_path = get_package_share_path('dofbot_info')
    default_model_path = urdf_package_path / 'urdf/dofbot.urdf'
    default_rviz_config_path = urdf_package_path / 'rviz/dofbot_info.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'], 
                                    description='Flag to enable gui')
    rviz_arg = DeclareLaunchArgument(name='rviz_config', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                            description='Path to robot urdf file relative to urdf_tutorial package')
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                    value_type=str)

    robot_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')))

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')])
    
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_node,
        rviz_node
    ])
