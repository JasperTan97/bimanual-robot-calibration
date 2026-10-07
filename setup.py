from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cone_calib'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARMS',
    maintainer_email='zhen.tan@epfl.ch',
    description='Base-to-base calibration of a bimanual setup via conical seats.',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # format: "<executable name> = <package>.<module>:<function>"
            'capture_node = cone_calib.capture_node:main',
            'solve = cone_calib.solve:main',
            'echo_sphere = cone_calib.echo_sphere:main',
        ],
    },
)
