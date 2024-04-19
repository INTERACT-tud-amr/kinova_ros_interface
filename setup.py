#!/usr/bin/env python

from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['kinova_ros_interface'],
    package_dir={'': 'src'},
)

setup(**setup_args)