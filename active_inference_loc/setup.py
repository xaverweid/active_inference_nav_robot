from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'active_inference_loc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

         (os.path.join('share', package_name, 'launch'),
            glob('launch/*_launch.py')),
    ],
    install_requires=[
            'setuptools',
            'numpy',
            'scipy',
            'scikit-learn',
            'matplotlib',
            'transforms3d',
            'numba',  
    ],    
    zip_safe=True,
    maintainer='xaver',
    maintainer_email='fxweidinger@googlemail.com',
    description='Active Inference based localization for mobile robots',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aic_node=active_inference_loc.aic_node:main',
            'belief_monitor = active_inference_loc.belief_monitor_node:main',
        ],
    },
    
)
