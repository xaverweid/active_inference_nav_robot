from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff_drive_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all directories
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xaverweid',
    maintainer_email='fxweidinger@googlemail.com',
    author='adoodevv',
    author_email='adoojonathan412@gmail.com',
    description='Differential drive robot with active inference localization',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'starting_pose_publisher.py=diff_drive_robot.scripts.starting_pose_publisher:main',
        ],
    },
)
