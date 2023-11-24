from setuptools import setup
import os
from glob import glob

package_name = 'ros2_term_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@test.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'starter=ros2_term_project.starter:main',
            'controller=ros2_term_project.controller:main',
            'line_follower=ros2_term_project.end_line_detector:main',
            'end_line_detector=ros2_term_project.line_follower:main',
            'stop_line_detector=ros2_term_project.stop_line_detector:main',
            'obstacle_detector=ros2_term_project.obstacle_detector:main',
            'actor_detect_processor=ros2_term_project.actor_detect_processor:main',
            'box_controller=ros2_term_project.box_controller:main'
        ],
    },
)
