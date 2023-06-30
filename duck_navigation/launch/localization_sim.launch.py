import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    package_name = "duck_navigation"
    
    nav2_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory(package_name), 'config', 'obs_map.yaml') #change to your map
    rviz_config_file = os.path.join(get_package_share_directory(package_name), "rviz", "navigation.rviz")
    use_rviz = LaunchConfiguration("rviz", default=False)

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=["-d", rviz_config_file],
        output= "screen",
        condition=IfCondition(use_rviz)
    )

    ])
