from setuptools import setup

package_name = 'cobot_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manipulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vibhu',
    maintainer_email='grownuphoon@gmail.com',
    description='MoveIt pick and place coordinator for Cobot 280',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'pick_place_coordinator = cobot_manipulation.pick_place_coordinator:main',
    ]},
)
