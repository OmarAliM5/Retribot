from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'navigation_pkg', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi5',
    maintainer_email='abouelnagamohamed33e3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_follower_node = navigation_pkg.path_follower_node:main',
            'manual_mode_node = navigation_pkg.manual_mode:main',
            'obstacle_avoidance_node = navigation_pkg.obstacle_avoid_node:main',
            'item_collect_node = navigation_pkg.item_collect_node:main',
            'mini_path_node = navigation_pkg.mini_path_node:main',
        ],
    },
)
