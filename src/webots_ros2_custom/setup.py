from setuptools import setup
import os
from glob import glob

package_name = 'webots_ros2_custom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            glob(os.path.join(package_name, 'launch', '*.py'))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivan@example.com',
    description='Custom Webots ROS2 interface for Shrimp robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'my_node = webots_ros2_custom.my_node:main'
        ],
    },
)
