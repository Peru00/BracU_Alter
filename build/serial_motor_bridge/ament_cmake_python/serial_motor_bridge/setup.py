from setuptools import find_packages
from setuptools import setup

setup(
    name='serial_motor_bridge',
    version='0.0.1',
    packages=find_packages(
        include=('serial_motor_bridge', 'serial_motor_bridge.*')),
)
