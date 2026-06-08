from setuptools import setup, find_packages

package_name = 'warehouse_digital_twin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/warehouse_twin.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Warehouse Digital Twin for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_simulator = warehouse_digital_twin.robot_simulator:main',
            'warehouse_controller = warehouse_digital_twin.warehouse_controller:main',
            'visualizer = warehouse_digital_twin.visualizer:main',
        ],
    },
)
