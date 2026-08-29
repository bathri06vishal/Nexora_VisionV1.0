import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('husky_outdoor_nav')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context)

    # Determine world path (shortcut aliases: forest, orchard, agriculture)
    if world_arg in ['orchard', 'cpr_orchard']:
        world_path = os.path.join(pkg_dir, 'worlds', 'cpr_orchard.world')
    elif world_arg in ['agriculture', 'cpr_agriculture']:
        world_path = os.path.join(pkg_dir, 'worlds', 'cpr_agriculture.world')
    elif world_arg == 'forest':
        world_path = os.path.join(pkg_dir, 'worlds', 'forest_terrain.world')
    elif os.path.isabs(world_arg) and os.path.exists(world_arg):
        world_path = world_arg
    else:
        world_path = os.path.join(pkg_dir, 'worlds', 'forest_terrain.world')

    # Environment variables for Gazebo models
    models_dir = os.path.join(pkg_dir, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{models_dir}:{existing_model_path}" if existing_model_path else models_dir

    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    gazebo_resource_path = f"{pkg_dir}:{models_dir}:{existing_resource_path}" if existing_resource_path else f"{pkg_dir}:{models_dir}"

    gazebo_env = {
        'GAZEBO_MODEL_PATH': gazebo_model_path,
        'GAZEBO_RESOURCE_PATH': gazebo_resource_path,
        'DISPLAY': os.environ.get('DISPLAY', ':1')
    }

    # Gazebo Process (unified gazebo for GUI, gzserver for headless)
    if gui == 'true':
        gazebo_cmd = [
            'gazebo', world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ]
    else:
        gazebo_cmd = [
            'gzserver', world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ]

    gazebo_proc = ExecuteProcess(
        cmd=gazebo_cmd,
        additional_env=gazebo_env,
        output='screen'
    )

    # Process Xacro URDF
    xacro_file = os.path.join(pkg_dir, 'urdf', 'husky.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': (use_sim_time == 'true')}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': (use_sim_time == 'true')}]
    )

    # Spawn Husky in Gazebo
    spawn_husky = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'husky',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen'
    )

    return [
        gazebo_proc,
        robot_state_publisher,
        joint_state_publisher,
        spawn_husky
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo GUI'),
        DeclareLaunchArgument('world', default_value='forest', description='World: forest | orchard | agriculture | custom path'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial X position'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial Y position'),
        DeclareLaunchArgument('spawn_z', default_value='0.25', description='Initial Z position'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial Yaw orientation'),
        OpaqueFunction(function=launch_setup)
    ])
