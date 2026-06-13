from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('robot_gui/templates/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi5',
    maintainer_email='pi5@example.com',
    description='Robot GUI package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = robot_gui.gui_node:main',
        ],
    },
)