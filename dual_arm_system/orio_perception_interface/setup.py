from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orio_perception_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@todo.todo',
    description='Swappable perception providers for the dual-arm system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pose_server = orio_perception_interface.grasp_pose_server:main',
            'label_pose_server = orio_perception_interface.label_pose_server:main',
        ],
    },
)
