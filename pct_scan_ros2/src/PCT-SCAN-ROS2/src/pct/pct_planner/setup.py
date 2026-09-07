from setuptools import find_packages, setup

package_name = 'pct_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='1523924956@qq.com',
    description='PCT global planner ported to ROS 2 Humble for PCT-SCAN-ROS2',
    license='GPL-2.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node = pct_planner.planner_node:main'
        ],
    },
)
