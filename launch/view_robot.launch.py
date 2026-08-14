import os

from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_orientation_publisher = LaunchConfiguration('enable_orientation_publisher')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('spm_robot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'view_robot.rviz')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_spm_joint_state_publisher = Node(
        package='spm_robot',
        executable='spm_joint_state_publisher',
        output='screen'
    )

    node_spm_orientation_publisher = Node(
        package='spm_robot',
        executable='spm_orientation_publisher',
        output='screen',
        condition=IfCondition(enable_orientation_publisher)
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'enable_orientation_publisher',
            default_value='false',
            description='Enable the orientation publisher'),

        node_robot_state_publisher,
        node_spm_joint_state_publisher,
        node_spm_orientation_publisher,
        node_rviz2
    ])
