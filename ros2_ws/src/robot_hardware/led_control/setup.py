from setuptools import setup, find_packages

package_name = 'led_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alesha',
    maintainer_email='alesha@todo.todo',
    description='LED control hardware node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_node = led_control.nodes.led_node:main',
        ],
    },
)
