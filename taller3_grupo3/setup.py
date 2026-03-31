import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'taller3_grupo3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grupo3',
    maintainer_email='grupo3@uniandes.edu.co',
    description='Taller 3 — Robot de clasificación de objetos con ROS2 y OpenCV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodos del Taller 2 que se conservan
            'robot_core     = taller3_grupo3.robot_core:main',
            'robot_teleop   = taller3_grupo3.robot_teleop:main',

            # Nodos nuevos del Taller 3
            'camera_node      = taller3_grupo3.camera_node:main',
            'vision_node      = taller3_grupo3.vision_node:main',
            'manipulator_core = taller3_grupo3.manipulator_core:main',
            'robot_fsm        = taller3_grupo3.robot_fsm:main',
        ],
    },
)
