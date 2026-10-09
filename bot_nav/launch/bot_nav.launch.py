import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('bot_nav'), 'config', 'nav2_params.yaml'
        ),
    )

    sim_pkg = get_package_share_directory('bot_sim')
    sim_launch = os.path.join(sim_pkg, 'launch', 'bot_sim.launch.py')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))],
    )

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments=[
            ('slam', 'True'),
            ('map', ''),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('params_file', LaunchConfiguration('params_file')),
            ('use_composition', 'False'),
        ],
    )
    return LaunchDescription([use_sim_time, params_file, sim, navigation])
