from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tm_robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='regina',
    maintainer_email='regina@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tm_arm_action_simulation = tm_robot_simulation.tm_arm_action_simulation:main',
            'tm_arm_loader = tm_robot_simulation.tm_arm_loader:main',
            'gripper_simulator = tm_robot_simulation.gripper_simulator:main',

        ],
    },
)
