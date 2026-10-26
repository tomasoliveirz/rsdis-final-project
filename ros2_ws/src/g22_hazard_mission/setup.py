from glob import glob

from setuptools import find_packages, setup

package_name = 'g22_hazard_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RSDIS Group 22',
    maintainer_email='group22@example.com',
    description='Simulated gas/radiation sensors and hazard visualisation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_sensor = g22_hazard_mission.hazard_sensor:main',
            'hazard_viz = g22_hazard_mission.hazard_viz:main',
            'uav_survey = g22_hazard_mission.uav_survey:main',
        ],
    },
)
