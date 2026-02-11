from setuptools import find_packages, setup

package_name = 'cleanwalker_perception'

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
    description='Perception nodes for CleanWalker CW-1',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'detection_node = cleanwalker_perception.detection_node:main',
            'depth_node = cleanwalker_perception.depth_node:main',
            'segmentation_node = cleanwalker_perception.segmentation_node:main',
        ],
    },
)
