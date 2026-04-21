from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'limelight_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Perception-only ROS 2 package for Limelight 3A',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'limelight_publisher = limelight_perception.limelight_publisher:main',
        ],
    },
)
