#!/usr/bin/env python2
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['rqt_control_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_control_gui']
)

setup(**setup_args)
