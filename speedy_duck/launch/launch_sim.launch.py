import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    package_name = 'speedy_duck'

    # Robot State Publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Gazebo Sim 
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'obstacles.world'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r ' + world_path}.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-topic', '/robot_description'],
        output='screen'
    )

    # ROS ↔ Gz Bridge (for /clock and /scan topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # Odom relay: /diff_cont/odom → /odom (Nav2 expects /odom) 
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diff_cont/odom', '/odom'],
        output='screen'
    )

    # ros2_control spawners (delayed to wait for gz_ros2_control plugin)
    diff_drive_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
        )]
    )

    joint_broad_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
        )]
    )

    # cmd_vel mapper
    cmd_vel_mapper = Node(
        package='speedy_duck',
        executable='cmd_vel_mapper',
        output='screen'
    )

    # RViz (with custom config)
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'view_bot.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz', default='false'))
    )

    return LaunchDescription([
        rsp,
        cmd_vel_mapper,
        gazebo,
        spawn_entity,
        bridge,
        odom_relay,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz,
    ])