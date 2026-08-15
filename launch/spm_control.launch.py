from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch RViz alongside the control stack.',
        ),
    ]

    gui = LaunchConfiguration('gui')

    # xacro-process the URDF into a robot_description parameter
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('spm_robot'),
            'description', 'robot.urdf.xacro',
        ]),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('spm_robot'),
        'config', 'spm_controllers.yaml',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('spm_robot'),
        'config', 'view_robot.rviz',
    ])

    # controller_manager, loaded with the mock hardware from robot_description
    # plus the controller definitions from spm_controllers.yaml
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )

    # Publishes /robot_description and, from joint_states, the TF tree
    # RViz needs to draw the robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    node_spm_joint_state_publisher = Node(
        package='spm_robot',
        executable='spm_joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Start RViz only once joint_state_broadcaster is up, so it doesn't come
    # up before /joint_states (and therefore TF) is being published
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Same reasoning for the position controller: claim the command
    # interfaces only after the broadcaster has successfully claimed the
    # state interfaces
    delay_forward_position_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_position_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        node_spm_joint_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_forward_position_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
