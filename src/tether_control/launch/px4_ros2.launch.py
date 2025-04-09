"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from tempfile import NamedTemporaryFile

def generate_launch_description():
    # Declare launch arguments
    bridge_config_path_arg = DeclareLaunchArgument('bridge_config_file', default_value='gazebo_bridge.yaml', description='Bridge config file path')
    package_name_arg = DeclareLaunchArgument('package_name', default_value='tether_control', description='Package name containing URDF file')
    tether_config_path_arg = DeclareLaunchArgument('tether_config_file', default_value='tether_config.yaml', description='Tether model config file path')

    return LaunchDescription([
        package_name_arg,
        bridge_config_path_arg,
        tether_config_path_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    bridge_config_file = LaunchConfiguration('bridge_config_file').perform(context)
    tether_config_file = LaunchConfiguration('tether_config_file').perform(context)
    package_name = LaunchConfiguration('package_name').perform(context)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bridge_config_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        bridge_config_file
    )

    tether_config_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        tether_config_file
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': True  # Add this line to enable simulation time
        }],
        output='screen',
        emulate_tty=True # coloured RCLCPP log, in case RCUTILS_COLORIZED_OUTPUT not set
    )

    tether_control_node = Node(
        package='tether_control',
        executable='tether_control_node',
        parameters=[tether_config_path],
        output='screen',
        emulate_tty=True # coloured RCLCPP log, in case RCUTILS_COLORIZED_OUTPUT not set
    )

    tether_model_node = Node(
        package='tether_control',
        executable='tether_model_node',
        parameters=[tether_config_path],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output='screen',
        emulate_tty=True # coloured RCLCPP log, in case RCUTILS_COLORIZED_OUTPUT not set
    )

    udp_process = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen')

    # px4_process = ExecuteProcess(
    #     cmd=['make', 'px4_sitl', 'gz_tethered_lin'],
    #     cwd='/home/yannis/Sas_lab/PX4-Autopilot',
    #     output='screen'
    # )

    return [
        udp_process,
        # px4_process,
        gazebo_bridge_node,
        tether_control_node,
        tether_model_node
    ]
