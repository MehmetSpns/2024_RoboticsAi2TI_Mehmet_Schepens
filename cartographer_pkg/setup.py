from setuptools import setup

package_name = 'cartographer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cartographer_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dogan',
    maintainer_email='mehmet.schepens@student.ehb.be',
    description='Package for TurtleBot3 cartographer and obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartographer_node = cartographer_pkg.cartographer_node:main',
        ],
    },
)
