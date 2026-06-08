from setuptools import setup
import os
from glob import glob

package_name = 'warehouse_digital_twin'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teacher',
    maintainer_email='teacher@school.edu.cn',
    description='智能仓储机器人数字孪生系统',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = warehouse_digital_twin.robot_simulator:main',
            'controller = warehouse_digital_twin.warehouse_controller:main',
            'visualizer = warehouse_digital_twin.visualizer:main',
            'web_visualizer = warehouse_digital_twin.web_visualizer:main',
        ],
    },
)
