import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import launch



def generate_launch_description():



    package_name='speedy_duck' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )
    
    rviz_config_file = os.path.join(get_package_share_directory(package_name), "rviz", "view_bot.rviz")
    use_rviz = LaunchConfiguration("rviz", default=False)
    rviz = Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=["-d", rviz_config_file],
        output= "screen",
        condition=IfCondition(use_rviz)
    )
    
    cmd_vel_mapper = Node(
        package="speedy_duck",
        executable="cmd_vel_mapper",
        output="screen"
    )


    
    
    
    
    ekf_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch them all!
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true'),
        rsp,
        cmd_vel_mapper,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        robot_localization_node,
        rviz,
    ])
    