from glob import glob
import os

from setuptools import find_packages, setup


package_name = "pct_scan_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "scipy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="PCT-SCAN-ROS2 maintainers",
    maintainer_email="iszhouxin@zju.edu.cn",
    description="Dynamic-layer PCT global-planner bridge for PCT-SCAN-ROS2.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "bridge_node = pct_scan_bridge.bridge_node:main",
            "pct_route_smoke = pct_scan_bridge.route_smoke:main",
        ],
    },
)
