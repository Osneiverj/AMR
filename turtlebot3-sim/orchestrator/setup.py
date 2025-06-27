from setuptools import setup

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name
        ]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system_launch.py']),
    ],
    install_requires=['rclpy'],
    entry_points={
        'console_scripts': [
            'orchestrator = orchestrator.main:main',
        ],
    },
)
