from setuptools import setup

package_name = 'web_led'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aleksei',
    maintainer_email='azarada552@gmail.com',
    description='Web interface for LED control using Flask and ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ⬇⬇⬇ ВАЖНО
            'web_led_app = web_led.app:main',
        ],
    },
)
