from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'unicycle_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['unicycle_brain', 'unicycle_brain.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Olejniczak',
    maintainer_email='radoslaw.olejniczak@student.put.poznan.pl',
    description='Unicycle control project',
    license='NONE',

    install_scripts='lib/' + package_name,

    data_files=[
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
    ],
    entry_points={
        'console_scripts': [
            'robot_model = unicycle_brain.robot_model:main',
            'trajectory_generator = unicycle_brain.trajectory_gen:main',
        ],
    },
)
