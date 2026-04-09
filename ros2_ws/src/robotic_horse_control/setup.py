from setuptools import setup
import os
from glob import glob

package_name = 'robotic_horse_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'animations'),
            glob('animations/*.json') + glob('animations/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Gait controller and Gazebo bringup for the Robotic Horse',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gait_node        = robotic_horse_control.gait_node:main',
            'force_node       = robotic_horse_control.force_node:main',
            'blendspace_node  = robotic_horse_control.blendspace_node:main',
        ],
    },
)
