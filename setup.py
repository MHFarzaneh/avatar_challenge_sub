from setuptools import setup
import os
from glob import glob

package_name = 'avatar_challenge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'shapes'), glob('shapes/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'moveit_commander',
    ],
    zip_safe=True,
    maintainer='Hamid Farzaneh',
    maintainer_email='your_email@example.com',
    description='A package for tracing 2D shapes and executing pose plans on xArm 7 using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shape_tracer_node = avatar_challenge.shape_tracer_node:main',
        ],
    },
)
