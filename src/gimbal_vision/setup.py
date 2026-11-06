from glob import glob

from setuptools import find_packages, setup

package_name = 'gimbal_vision'

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
    description='Visual orientation error e_R from camera images.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'vision_node = gimbal_vision.vision_node:main',
        ],
    },
)
