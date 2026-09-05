import os
from glob import glob
from setuptools import setup
package_name = 'gz_course_week_1'
def package_files(root_dir, install_root):
    data = []
    for dirpath, _, filenames in os.walk(root_dir):
        if not filenames:
            continue
        files = [os.path.join(dirpath, f) for f in filenames]
        rel_dir = os.path.relpath(dirpath, root_dir)
        dest_dir = os.path.join(install_root, rel_dir) if rel_dir != '.' else install_root
        data.append((dest_dir, files))
    return data
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'),
     glob('config/*.yaml')),
    (os.path.join('share', package_name, 'worlds'),
     glob('worlds/*.sdf')),
    (os.path.join('share', package_name, 'rviz'),
     glob('rviz/*.rviz')),
]
data_files += package_files(
    'models/turtlebot3_burger',
    os.path.join('share', package_name, 'models', 'turtlebot3_burger')
)
data_files += package_files(
    'models/turtlebot3_common',
    os.path.join('share', package_name, 'models', 'turtlebot3_common')
)
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keith_C',
    maintainer_email='k.currier@ufl.edu',
    description='Gazebo Harmonic + ROS 2 Humble course package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = gz_course_week_1.teleop_node:main',
            'cbf_safety_node = gz_course_week_1.cbf_safety_node_pos:main',
            'spiral_tracker = gz_course_week_1.spiral_tracker_node:main',
            'trajectory_recorder = gz_course_week_1.trajectory_recorder_node:main',
            'ground_truth_pose = gz_course_week_1.ground_truth_pose_node:main',
        ],
    },
)