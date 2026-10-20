import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'esekf19_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='AUV Navigation Team',
    maintainer_email='auv_team@domain.com',
    description='ROS 2 19-State ES-EKF Navigation Node with Boresight & Scale Factor Calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esekf19_node = esekf19_nav.esekf19_node:main',
        ],
    },
)
