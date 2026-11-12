from glob import glob

from setuptools import find_packages, setup

package_name = 'base_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Häfner',
    maintainer_email='haefner.maxim@gmail.com',
    description='Command sources for the Kobuki base, run on the gimbal Pi: keyboard teleop and excitation sequences.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard = base_teleop.keyboard_node:main',
            'excitation = base_teleop.excitation_node:main',
        ],
    },
)
