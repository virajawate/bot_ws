import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    bot_nav_pkg = get_package_share_directory('bot_nav')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    ld.add_action(use_sim_time)
    
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bot_nav_pkg, 'config', 'nav2_params.yaml'
        ),
    )
    ld.add_action(params_file)

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bot_nav_pkg, 'config', 'batman.yaml')
    )
    ld.add_action(map_yaml_file)
    rviz_config = os.path.join(bot_nav_pkg, 'config', 'nav.rviz')
    rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_config,
        description='Full path to the RVIZ config file to use',
    )
    ld.add_action(rviz_config_file)
    
    sim_pkg = get_package_share_directory('bot_sim')
    sim_launch = os.path.join(sim_pkg, 'launch', 'bot_sim.launch.py')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time'),), 
            ('rviz_config_file', LaunchConfiguration('rviz_config_file'))
        ],
    )

    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments=[
            ('slam', 'False'),
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('params_file', LaunchConfiguration('params_file')),
            ('use_composition', 'False'),
        ],
    )
    return LaunchDescription([use_sim_time, params_file, rviz_config_file, map_yaml_file, sim, navigation])
