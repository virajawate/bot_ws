import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg_bringup = get_package_share_directory('bot_bringup')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'bot_one.urdf')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(pkg_bringup, 'worlds', 'empty_with_sensors.sdf')
    robot_description = Command(['xacro ', urdf_file])
    use_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )
    ld.add_action(use_time)
    gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value='-r ' + world_file,
        description='Arguments passed to Ignition Gazebo',
    )
    ld.add_action(gz_args)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    ld.add_action(rsp)

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    ld.add_action(jsp)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', LaunchConfiguration('gz_args'))],
    )
    ld.add_action(gz_sim)

    bot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description', 
            '-name', 'bot_one', 
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    ld.add_action(bot_spawn)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )
    ld.add_action(bridge)

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        parameters=[{'override_frame_id': 'laser'}],
        output='screen',
    )
    ld.add_action(scan_bridge)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'bot.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    ld.add_action(rviz)

    return ld