from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('nav2/launch/*.py')),
        (os.path.join('share', package_name, 'params'),
            glob('nav2/params/*.yaml')),
        (os.path.join('share', package_name, 'map'),
            glob('nav2/map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anirudh',
    maintainer_email='anirudh110106@gmail.com',
    description='Robot system with odometry and SLAM',
    license='Apache License 2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'odom = nav2.base_odom:main',
            'laser_node = nav2.laser:main',
        ],
    },
)
