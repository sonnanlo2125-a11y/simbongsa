from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_control = robot_control.robot_control:main',
            'test_robot_control = robot_control.test_robot_control:main',
            'test1_robot_control = robot_control.test_robot_control1:main',
            'test2_robot_control = robot_control.test_robot_control2:main',
            'mock_yolo = robot_control.mock_yolo_server:main',
            'mock_voice = robot_control.mock_voice:main',
        ],
    },
)
