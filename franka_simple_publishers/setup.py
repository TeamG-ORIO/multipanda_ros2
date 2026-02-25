from setuptools import find_packages, setup

package_name = 'franka_simple_publishers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jin-mirmi',
    maintainer_email='s.bien@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_joints = franka_simple_publishers.move_joints:main',
            'move_to_pose = franka_simple_publishers.move_to_pose:main',
        ],
    },
)
