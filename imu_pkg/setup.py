from setuptools import find_packages, setup

package_name = 'imu_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = imu_pkg.imu_node:main',
            'position_pid_node = imu_pkg.position_pid_node:main',
            ##'imu_position_node = imu_pkg.imu_position_node:main',
            ##'ekf_node = imu_pkg.ekf_node:main',
        ],
    },
)
