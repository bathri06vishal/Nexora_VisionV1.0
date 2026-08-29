from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    costmap_node = Node(
        package='husky_outdoor_nav',
        executable='elevation_costmap_25d_node',
        name='elevation_costmap_25d_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'grid_width': 20.0,
            'grid_height': 20.0,
            'resolution': 0.10,
            'max_step_height': 0.18,
            'max_slope_deg': 22.0,
            'robot_frame': 'base_footprint',
            'map_frame': 'odom',
            'input_pointcloud': '/camera/depth/points',
            'semantic_pointcloud': '/perception/semantic_pointcloud'
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        costmap_node
    ])
