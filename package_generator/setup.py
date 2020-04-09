# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
"""
@package package_generator
@file setup.py
@author Anthony Remazeilles
@brief standard python package setup

Copyright (C) 2017 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
SETUP_ARGS = generate_distutils_setup(
    packages=['package_generator'],
    package_dir={'': 'src'})

setup(**SETUP_ARGS)
