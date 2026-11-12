from glob import glob

from setuptools import find_packages, setup

package_name = 'base_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/systemd', glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Häfner',
    maintainer_email='haefner.maxim@gmail.com',
    description='Robot Pi relay: forwards /cmd_vel to the Kobuki driver at a fixed rate and echoes the executed command on /base_command.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_command_node = base_command.base_command_node:main',
        ],
    },
)
