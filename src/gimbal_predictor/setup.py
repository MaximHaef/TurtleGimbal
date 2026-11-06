from glob import glob

from setuptools import find_packages, setup

package_name = 'gimbal_predictor'

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
    description='Predicts base angular velocity from robot commands (physical model + flow-matching residuals).',
    license='TODO',
    entry_points={
        'console_scripts': [
            'predictor_node = gimbal_predictor.predictor_node:main',
        ],
    },
)
