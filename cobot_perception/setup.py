from setuptools import setup

package_name = 'cobot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vibhu',
    maintainer_email='grownuphoon@gmail.com',
    description='RGB-D object detection and tracking for Cobot 280',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'rgbd_object_tracker = cobot_perception.rgbd_object_tracker:main',
            'conveyor_manager = cobot_perception.conveyor_manager:main',
        ],
    },
)
