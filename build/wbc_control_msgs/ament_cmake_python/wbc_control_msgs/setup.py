from setuptools import find_packages
from setuptools import setup

setup(
    name='wbc_control_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('wbc_control_msgs', 'wbc_control_msgs.*')),
)
