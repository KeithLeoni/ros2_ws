from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pose_estimator_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),  # Automatically include all msg files
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'rosidl_default_runtime'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pt_inference_node = pose_estimator_pkg.pt_inference_node:main',
            'visualization_node = pose_estimator_pkg.visualization_node:main',
            'pose_estimator_node = pose_estimator_pkg.pose_estimator:main',
            'icp_pose_estimator_node = pose_estimator_pkg.icp_pose_estimator:main',
            'detection_result_republisher = pose_estimator_pkg.detection_result_republisher:main',
        ],
    },
)

