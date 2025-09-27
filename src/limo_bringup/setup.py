from setuptools import setup
from glob import glob
import os

package_name = 'limo_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*.launch.py'))),
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
     ['resource/' + package_name]),
    (os.path.join('share', package_name), ['package.xml']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Jazzy bringup for Agilex LIMO (RViz + Gazebo Sim)',
    license='BSD-3-Clause',
)
