from glob import glob
import os

from setuptools import setup

package_name = "go2_patrol"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Autonomous patrol package for Unitree Go2 robot using Nav2",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "patrol_node = go2_patrol.patrol_node:main",
        ],
    },
)
