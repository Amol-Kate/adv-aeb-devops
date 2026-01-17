from setuptools import setup

package_name = 'aeb_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amol',
    maintainer_email='you@example.com',
    description='Automatic Emergency Braking ROS2 nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = aeb_system.camera_node:main',
            'pedestrian_detector = aeb_system.pedestrian_detector:main',
            'aeb_decision_node = aeb_system.aeb_decision_node:main',
            'vehicle_control_node = aeb_system.vehicle_control_node:main',
        ],
    },
)
