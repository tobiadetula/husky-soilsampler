from setuptools import find_packages, setup

package_name = 'clearpath_waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Offboard waypoint navigation for Clearpath A300',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_commander = clearpath_waypoint_nav.waypoint_commander:main',
        ],
    },
)
