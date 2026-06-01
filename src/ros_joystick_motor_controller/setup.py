from setuptools import find_packages, setup

package_name = 'ros_joystick_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tobi',
    maintainer_email='oluwatobi.adetula1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_motor_node = ros_joystick_motor_controller.joystick_motor_node:main'
        ],
    },
)
