from setuptools import find_packages, setup

package_name = 'lidar_mediapipe_mehmet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/lidar_mediapipe_mehmet_launch_file.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dogan',
    maintainer_email='mehmet.schepens@student.ehb.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesturecontrol = lidar_mediapipe_mehmet.gesturecontrol:main',
            'robotcontrol = lidar_mediapipe_mehmet.robotcontrol:main',
        ],
    },
)
