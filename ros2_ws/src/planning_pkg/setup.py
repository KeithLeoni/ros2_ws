from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'planning_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning_node = planning_pkg.planning:main',
            'move_a_to_b_server = planning_pkg.move_a_to_b_server:main',
            'block_frame_publisher = planning_pkg.block_frame_publisher:main',
        ],
    },
)
