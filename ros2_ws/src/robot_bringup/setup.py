from setuptools import setup
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aleksei Zariada',
    maintainer_email='azarada552@gmail.com',
    description='Bringup package for ROS2 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_bringup_nop = robot_bringup.nop:main',  # фиксируем ноду
        ],
    },
)
