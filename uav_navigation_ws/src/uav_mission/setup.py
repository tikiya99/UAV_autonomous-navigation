from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thasinduwickrama',
    maintainer_email='thasinduwickrama12@gmail.com',
    description='Mission planning and execution for autonomous UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor_node = uav_mission.mission_executor_node:main',
        ],
    },
)
