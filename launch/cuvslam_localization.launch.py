import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_outdoor_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_isaac_cuvslam = LaunchConfiguration('use_isaac_cuvslam', default='false')
    use_ekf = LaunchConfiguration('use_ekf', default='false')

    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf_outdoor.yaml')
    cuvslam_config_path = os.path.join(pkg_dir, 'config', 'cuvslam_params.yaml')

    # Optional robot_localization EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odom_filtered')],
        condition=IfCondition(use_ekf)
    )

    # Isaac ROS cuVSLAM Node
    cuvslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        name='visual_slam_node',
        output='screen',
        parameters=[cuvslam_config_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
            ('/stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
            ('/visual_slam/imu', '/imu/data')
        ],
        condition=IfCondition(use_isaac_cuvslam)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('use_isaac_cuvslam', default_value='false', description='Enable Isaac ROS cuVSLAM hardware acceleration'),
        DeclareLaunchArgument('use_ekf', default_value='false', description='Enable robot_localization EKF'),
        ekf_node,
        cuvslam_node
    ])
