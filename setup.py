import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'husky_outdoor_nav'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ]
    
    # Recursively add models directory (meshes, configs, sdfs)
    for root, dirs, files in os.walk('models'):
        if files:
            dest_dir = os.path.join('share', package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((dest_dir, file_paths))
            
    # Recursively add data directory (RUGD sample dataset)
    if os.path.exists('data'):
        for root, dirs, files in os.walk('data'):
            if files:
                dest_dir = os.path.join('share', package_name, root)
                file_paths = [os.path.join(root, f) for f in files]
                data_files.append((dest_dir, file_paths))
            
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Engineer',
    maintainer_email='user@todo.todo',
    description='Husky UGV Outdoor Navigation in Gazebo with RUGD, YOLO11n-seg, cuVSLAM, 2.5D Costmap, Smac Hybrid-A*, and RPP',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo11n_traversability_node = husky_outdoor_nav.yolo11n_traversability_node:main',
            'rugd_dataset_player_node = husky_outdoor_nav.rugd_dataset_player_node:main',
            'elevation_costmap_25d_node = husky_outdoor_nav.elevation_costmap_25d_node:main',
            'goal_patrol_node = husky_outdoor_nav.goal_patrol_node:main',
        ],
    },
)
