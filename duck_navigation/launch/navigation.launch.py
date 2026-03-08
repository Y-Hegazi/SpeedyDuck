import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    package_name = "duck_navigation"

    nav2_yaml         = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    map_file          = os.path.join(get_package_share_directory(package_name), 'config', 'my_map.yaml')
    rviz_config_file  = os.path.join(get_package_share_directory(package_name), 'rviz', 'navigation.rviz')
    controller_yaml   = os.path.join(get_package_share_directory(package_name), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'bt_navigator.yaml')
    planner_yaml      = os.path.join(get_package_share_directory(package_name), 'config', 'planner_server.yaml')
    recovery_yaml     = os.path.join(get_package_share_directory(package_name), 'config', 'recovery.yaml')
    bt_xml_path       = os.path.join(get_package_share_directory(package_name), 'config', 'behavior.xml')

    use_rviz = LaunchConfiguration('rviz', default='false')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml,
                        {'use_sim_time': True},
                        {'default_nav_to_pose_bt_xml': bt_xml_path}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(use_rviz)
        ),

        # Publish initial pose at origin after 8s so AMCL activates
        # AMCL needs this to publish map→odom transform.
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'
                ],
                output='screen'
            )]
        ),
    ])
