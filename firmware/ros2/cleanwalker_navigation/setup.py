from setuptools import find_packages, setup

package_name = 'cleanwalker_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CleanWalker Robotics',
    maintainer_email='engineering@cleanwalkerrobotics.com',
    description='Navigation and SLAM nodes for CleanWalker CW-1',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'slam_node = cleanwalker_navigation.slam_node:main',
            'path_planner_node = cleanwalker_navigation.path_planner_node:main',
        ],
    },
)
