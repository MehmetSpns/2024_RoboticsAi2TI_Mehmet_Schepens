from setuptools import setup
import os
from glob import glob

package_name = 'lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Only install package.xml in the share directory
        (os.path.join('share', package_name), ['package.xml']),
        # Only install launch files in the launch subdirectory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Marker file for ament indexing
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [f'resource/{package_name}']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of my lidar package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = lidar_pkg.lidar:main'
        ],
    },
)
