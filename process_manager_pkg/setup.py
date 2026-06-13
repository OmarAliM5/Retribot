from setuptools import setup
import os
from glob import glob

package_name = 'process_manager_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='pi5',
    maintainer_email='pi5@localhost',
    description='Process Manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'process_manager_node = process_manager_pkg.process_manager_node:main',
        ],
    },
)