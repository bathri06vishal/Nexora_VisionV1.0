import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_outdoor_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration('model_path', default='yolo11n-seg.pt')
    conf_thresh = LaunchConfiguration('conf_thresh', default='0.25')

    yolo_node = Node(
        package='husky_outdoor_nav',
        executable='yolo11n_traversability_node',
        name='yolo11n_traversability_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
            'confidence_threshold': conf_thresh,
            'image_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'publish_pointcloud': True,
            'enable_cuda': True
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('model_path', default_value='yolo11n-seg.pt', description='YOLOv11 segmentation weights'),
        DeclareLaunchArgument('conf_thresh', default_value='0.25', description='Confidence threshold'),
        yolo_node
    ])
