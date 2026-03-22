from setuptools import setup
import os
from glob import glob

package_name = 'assignment_ws'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srx',
    maintainer_email='srihaas.x@gmail.com',
    description='PX4 ROS2 SITL assignment workspace',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_subscriber    = assignment_ws.state_subscriber:main',
            'perception_node     = assignment_ws.perception_node:main',
            'waypoint_navigator  = assignment_ws.waypoint_navigator:main',
        ],
    },
)
