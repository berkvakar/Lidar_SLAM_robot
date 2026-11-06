import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # The web page is read at runtime from the package share directory.
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkvakar',
    maintainer_email='berkvakar1@gmail.com',
    description='Live LiDAR scan viewer: streams /scan to a web page served from the robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_web_server = my_robot_viz.scan_web_server:main',
        ],
    },
)
