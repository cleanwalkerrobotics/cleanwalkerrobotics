import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cleanwalker_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CleanWalker Robotics',
    maintainer_email='engineering@cleanwalkerrobotics.com',
    description='Robot description and visualization for CleanWalker CW-1',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [],
    },
)
