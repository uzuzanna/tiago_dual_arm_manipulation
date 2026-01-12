from setuptools import setup
import os
from glob import glob

package_name = 'tiago_target_pose'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zuzanna Urbaniak',
    maintainer_email='zuzanna.urbaniak.1@student.put.poznan.pl',
    description='Interface package for grasp target publishing. Calculates the final end-effector pose based on perception centroids and applies kinematic offsets for the TIAGo Pro gripper.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'publish_centroid_pose = tiago_target_pose.publish_centroid_pose:main',
        ],
    },
)

