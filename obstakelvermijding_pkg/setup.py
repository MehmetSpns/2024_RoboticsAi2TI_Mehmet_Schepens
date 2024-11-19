from setuptools import setup
from glob import glob
import os

package_name = 'obstakelvermijding_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy launch files to the correct location
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehmet',
    maintainer_email='mehmet@example.com',
    description='ObstakelVermijding',
    license='Apache License 2.0',
    tests_require=['pytest'],
  entry_points={
    'console_scripts': [
        'obstakelvermijding = obstakelvermijding_pkg.obstakelvermijding:main',
    ],
},

)
