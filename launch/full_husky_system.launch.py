import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_outdoor_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    world = LaunchConfiguration('world', default='forest')

    # 1. Gazebo Husky Simulation (supports 'forest', 'orchard', 'agriculture', or custom .world)
    husky_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'husky_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'gui': gui, 'world': world}.items()
    )

    # 2. YOLO11n-seg Semantic Traversability Node
    yolo_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'yolo11n_perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. 2.5D Costmap Generation
    costmap_25d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'costmap_25d.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Localization / EKF / cuVSLAM
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'cuvslam_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Nav2 Stack with Smac Hybrid-A* & Regulated Pure Pursuit (delayed for Gazebo bringup)
    nav2_stack = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'nav2_outdoor.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 6. RViz2 Visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz_outdoor_nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('world', default_value='forest', description='Select World: forest | orchard | agriculture | custom path'),
        husky_sim,
        yolo_perception,
        costmap_25d,
        localization,
        nav2_stack,
        rviz_node
    ])
