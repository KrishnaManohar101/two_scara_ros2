from setuptools import find_packages
from setuptools import setup

setup(
    name='two_scara_collaboration',
    version='0.0.1',
    packages=find_packages(
        include=('two_scara_collaboration', 'two_scara_collaboration.*')),
)
