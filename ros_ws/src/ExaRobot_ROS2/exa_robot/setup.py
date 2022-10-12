from setuptools import setup
import os
from glob import glob

package_name = 'exa_robot'
launch_path = 'exa_robot/launch'
config_path = 'exa_robot/config'
map_path = 'exa_robot/map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', launch_path), glob('launch/*')),
        (os.path.join('share', config_path), glob('config/*')),
        (os.path.join('share', map_path), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='seyeon@seoultech.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = exa_robot.exa_node:main'
        ],
    },
)
