from glob import glob

from setuptools import find_packages, setup

package_name = 'gimbal_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Häfner',
    maintainer_email='haefner.maxim@gmail.com',
    description='Fixed-rate gimbal control loop: proposal weighting, feedforward and feedback joint-rate commands.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'control_node = gimbal_control.control_node:main',
        ],
    },
)
