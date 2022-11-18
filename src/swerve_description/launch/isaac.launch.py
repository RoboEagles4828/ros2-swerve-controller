import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = True

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('swerve_description'))
    xacro_file = os.path.join(pkg_path,'urdf', 'robots','swerve.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joystick_file = os.path.join(pkg_path, 'config', 'xbox-holonomic.config.yaml')
    rviz_file = os.path.join(pkg_path, 'config', 'view.rviz')


    swerve_description_config = xacro.process_file(xacro_file)
    
    # Save Built URDF file to Description Directory
    swerve_description_xml = swerve_description_config.toxml()
    source_code_path = os.path.abspath(os.path.join(pkg_path, "../../../../src/swerve_description"))
    urdf_save_path = os.path.join(source_code_path, "swerve.urdf")
    with open(urdf_save_path, 'w') as f:
        f.write(swerve_description_xml)

    
     # Create a robot_state_publisher node
    params = {'robot_description': swerve_description_xml, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Starts ROS2 Control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': swerve_description_xml, 'use_sim_time': True }, controllers_file],
        output="screen",
    )


    # Starts ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    


    #Starts ROS2 Control Swerve Drive Controller
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "-c", "/controller_manager"],
    )
    swerve_drive_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )


    # Start Rviz2 with basic view
    run_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{ 'use_sim_time': True }],
        name='isaac_rviz2',
        output='screen',
        arguments=[["-d"], [rviz_file]],
    )


    # run_rviz2 = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_file],
    #     output='screen'
    # )
    rviz2_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[run_rviz2_node],
        )
    )


    # Start Joystick Node
    joy = Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])


    # Start Teleop Node to translate joystick commands to robot commands
    joy_teleop = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node', 
        parameters=[joystick_file],
        remappings={('/cmd_vel', '/swerve_controller/cmd_vel_unstamped')}
        )


    # Launch!
    return LaunchDescription([
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_delay,
        rviz2_delay,
        joy,
        joy_teleop
    ])