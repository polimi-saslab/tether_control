"""Launch a Gazebo simulation spawning a PX4 drone communicating over ROS2."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from tempfile import NamedTemporaryFile

def generate_launch_description():
    # Declare launch arguments
    # urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='custom_r2d2.urdf', description='URDF file name')
    bridge_config_file_arg = DeclareLaunchArgument('bridge_config_file', default_value='gazebo_bridge.yaml', description='Bridge config file name')
    package_name_arg = DeclareLaunchArgument('package_name', default_value='tether_control', description='Package name containing URDF file')
    # gazebo_world_arg = DeclareLaunchArgument('gazebo_world', default_value='gazebo_simulation_scene2_world.sdf', description='Gazebo world file name')


    return LaunchDescription([
        # urdf_file_arg,
        bridge_config_file_arg,
        package_name_arg,
        # gazebo_world_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    # urdf_file = LaunchConfiguration('urdf_file').perform(context)
    bridge_config_file = LaunchConfiguration('bridge_config_file').perform(context)
    # gazebo_world_file = LaunchConfiguration('gazebo_world').perform(context)
    package_name = LaunchConfiguration('package_name').perform(context)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bridge_config_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        bridge_config_file
    )

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file_path,
            'use_sim_time': True  # Add this line to enable simulation time
        }],
        output='screen'
    )

    udp_process = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen')
    
    # ExecuteProcess(
    #     cmd=['/home/yannis/Downloads/QGroundControl.AppImage'],
    #     output='screen'),

    px4_process = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_tether_cable'],
        cwd='/home/yannis/Sas_lab/PX4-Autopilot',
        output='screen'
    )

    return [
        # file_server2_node,
        # gazebo_node,
        # spawn_entity,
        gazebo_bridge_node,
        udp_process,
        px4_process
    ]
