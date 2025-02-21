import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch.conditions import IfCondition
import os
from launch_ros.descriptions import ParameterValue
from launch.actions import TimerAction
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('butler_robot')
    
    # Define paths
    xacro_file = os.path.join(pkg_share,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'cafe.world')
    # sdf_path = os.path.join(pkg_share, 'models/urdf/curio/model.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joy_params = os.path.join(pkg_share, 'config', 'joystick.yaml')
    joy_node = launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
    )

    teleop_node = launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )
    
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = launch_ros.actions.Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    spawn_entity = launch_ros.actions.Node(
        condition=IfCondition(use_sim_time),
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'butler_robot', '-topic', 'robot_description'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    controller_manager_node = launch_ros.actions.Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    

    diff_drive_spawner = TimerAction(
        period=5.0,  # Delay to allow controller_manager_node to initialize
        actions=[
            launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
            )
        ]
    )

    joint_broad_spawner = TimerAction(
        period=6.0,  # Delay to allow diff_drive_spawner to complete
        actions=[
            launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
            )
        ]
    )

    imu_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_config = launch_ros.actions.Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[imu_params_file],
                remappings=[("odom", LaunchConfiguration('odom_topic', default='/odom'))]

            )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_file,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_ros2_control', default_value='False',
                                         description='Enable ROS 2 Control'),
        launch.actions.ExecuteProcess(condition=IfCondition(use_sim_time), cmd=['gazebo', '--verbose', '-s',
                                            'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
                                        output='screen'),
        # controller_manager_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joy_node,
        teleop_node,
        twist_mux,
        controller_manager_node,
        diff_drive_spawner,
        joint_broad_spawner,
        spawn_entity,
        rviz_node,
        # ekf_config,
    ])