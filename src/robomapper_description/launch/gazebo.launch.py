from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def launch_setup(context, *args, **kwargs):
    # Get values from launch arguments
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = use_sim_time_str.lower() == 'true'
    
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world_name = LaunchConfiguration('world').perform(context)

    # Get paths for robot description
    robot_description_pkg_path = FindPackageShare('robomapper_description')

    # Path to the URDF file
    urdf_path = PathJoinSubstitution([
        robot_description_pkg_path,
        'urdf',
        'robomapper_bot.urdf'
    ]).perform(context)

    # Path to the controller configuration files
    pkg_share_dir = get_package_share_directory('robomapper_description')
    diff_drive_controller_config_path = os.path.join(pkg_share_dir, 'config', 'diff_drive_controller.yaml')
    controller_manager_config_path = os.path.join(pkg_share_dir, 'config', 'controller_manager.yaml') # Not directly used by plugin but good to reference if needed

    # --- Generate Robot Description (URDF) ---
    robot_description_content = Command(['xacro ', urdf_path]).perform(context)
    robot_description = {'robot_description': robot_description_content}

    # --- Nodes Definition ---

    # 1. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_bool}],
    )

    # 2. Gazebo Launch (using the standard gazebo_ros launch file)
    gazebo_ros_share_dir = FindPackageShare('gazebo_ros')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_ros_share_dir,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': [world_name, '.world'],
            'verbose': 'true',
            'gui': 'true',
            'use_sim_time': use_sim_time_str
        }.items(),
    )

    # 3. Spawn Entity Node (to spawn your robot in Gazebo)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', robot_name,
                   '-x', '0.0', '-y', '0.0', '-z', '0.05', # Spawn at base_link's origin Z for ground clearance
                   '-Y', '1.5708', # <-- FIX: Rotate 90 degrees (pi/2 radians) around Z (Yaw)
                   '-timeout', '60.0'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_bool}],
    )

    # 4. Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            # We don't need to pass --param-file here as it's defined in controller_manager.yaml
        ],
        output="screen",
    )

    # 5. Differential Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', "/controller_manager",
            '--controller-manager-timeout', '50',
            "--param-file", diff_drive_controller_config_path # Pass the diff_drive_controller's params here
        ],
        output="screen",
    )

    return [
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='robomapper_bot',
            description='Name of the robot to spawn in Gazebo',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Gazebo world to launch',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
