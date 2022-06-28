import glob
import os

from setuptools import setup

package_name = 'execute_rosbag_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'launch',
        'setuptools',
    ],
    zip_safe=True,
    author='Franco Cipollone',
    author_email='franco.c@ekumenlabs.com',
    maintainer='Franco Cipollone',
    maintainer_email='franco.c@ekumenlabs.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description=(
        'Launch file example for executing rosbag process.'
    ),
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = execute_rosbag_py.talker:main',
        ],
    },
)
