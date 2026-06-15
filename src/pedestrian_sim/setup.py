import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pedestrian_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hassan',
    maintainer_email='hsn07pk@gmail.com',
    description='Random-walk pedestrian simulator for Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pedestrian_sim_node = pedestrian_sim.pedestrian_sim_node:main',
        ],
    },
)
