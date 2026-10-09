import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	ld = LaunchDescription()
	pkg_bringup = get_package_share_directory('bot_bringup')
	bringup_launch = os.path.join(pkg_bringup, 'launch', 'bot_bringup.launch.py')
	use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
	bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(bringup_launch),
		launch_arguments=[
			('use_sim_time', LaunchConfiguration('use_sim_time')),
		],
	)
	ld.add_action(use_sim_time)
	ld.add_action(bringup)

	drive_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
			'/model/bot_one/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
			'/model/bot_one/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
			'/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
		],
		remappings=[
			('/model/bot_one/cmd_vel', '/cmd_vel'),
			('/model/bot_one/odometry', '/odom'),
		],
		output='screen',
	)
	ld.add_action(drive_bridge)

	return ld
