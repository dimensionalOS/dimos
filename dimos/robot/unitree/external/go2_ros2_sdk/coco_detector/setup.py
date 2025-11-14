import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'coco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julian',
    maintainer_email='julian.w.francis@gmail.com',
    description='COCO detector',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "coco_detector_node = coco_detector.coco_detector_node"
        ],
    },
)
