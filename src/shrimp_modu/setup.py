from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'shrimp_modu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='legacy',
    maintainer_email='ivan.torres.ortiz4487@gmail.com',
    description='SHRIMP MODU ROS2 PACKAGE',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shrimp_controller = shrimp_modu.shrimp_controller:main',
            'pure_pursuit = shrimp_modu.pure_pursuit_node:main',
            'shrimp_odometry = shrimp_modu.odometry_full:main',
            'servo_debug = shrimp_modu.servo_debug:main',
            'odom_tf_node = shrimp_modu.odom_tf_node:main',
            'navigate_to_pose_approach = shrimp_modu.navigate_to_pose_approach:main',
            'nav_debug_logger = shrimp_modu.nav_debug_logger:main',
            'nav_metrics_logger = shrimp_modu.nav_metrics_logger:main',
        ],
    },
)
