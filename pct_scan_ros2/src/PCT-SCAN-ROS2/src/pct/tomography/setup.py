import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tomography'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('lib/' + package_name, ['scripts/tomography_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='1523924956@qq.com',
    description='PCT point-cloud tomography ported to ROS 2 Humble',
    license='GPL-2.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
