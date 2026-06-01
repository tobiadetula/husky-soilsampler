from setuptools import find_packages, setup

package_name = 'clearpath_soil_sampler'

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
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sample_action_server = clearpath_soil_sampler.sample_action_server:main',
            'mock_action_server = clearpath_soil_sampler.mock_action_server:main',
            'mission_commander = clearpath_soil_sampler.mission_commander:main',
            'simple_navigator = clearpath_soil_sampler.simple_navigator:main',
        ],
    },
)
