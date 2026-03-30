from setuptools import find_packages, setup

package_name = 'aura_s7'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nihal Sanjay Seth',
    maintainer_email='seth.nihal.work@gmail.com',
    description='ROS2 trajectory tracking controller for UR5 using MATLAB-generated optimal trajectories (Project AURA Sprint 7)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_planner = aura_s7.trajectory_planner_node:main',
            'controller_node = aura_s7.controller_node:main',
        ],
    },
)