import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directories
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # World path
    world = os.path.expanduser(
        '~/ros2_ws/src/gazebo_models_worlds_collection/worlds/test_zone.world'
    )

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='-0.5')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav2, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map yaml file'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav2, 'params', 'nav2_params.yaml'),
        description='Full path to param file'
    )
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    # Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn robot
    spawn_tb3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # Build LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_tb3_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
