from setuptools import find_packages, setup

package_name = 'orio_rqt_dual_arm_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@todo.todo',
    description='rqt GUI plugin for the dual-arm pick-label-place system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_arm_gui = orio_rqt_dual_arm_gui.dual_arm_gui:main',
        ],
    },
)
