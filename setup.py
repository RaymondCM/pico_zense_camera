#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rasberry_data_collection', 'rasberry_data_collection_pkg', 'database_manager', 'odroid'],
    package_dir={'': 'src'},
    requires=['numpy', 'opencv-python', 'PyYaml', 'rospkg', 'tf', 'Pillow', 'smbus']
)

setup(**setup_args)


