from setuptools import find_packages, setup

package_name = 'mecanum_controller_pkg'

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
    maintainer='pi5',
    maintainer_email='pi5@todo.todo',
    description='ROS2 mecanum wheel controller and ESP32 serial bridge',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mecanum_controller_node = mecanum_controller_pkg.mecanum_controller_node:main',
            'mecanum_odometry_node = mecanum_controller_pkg.mecanum_odometry_node:main',
        ],
    },
)
