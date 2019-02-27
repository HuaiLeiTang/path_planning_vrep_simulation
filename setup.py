#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['path_planning_vrep_simulation',
              'path_planner',
              'vrep_communicator'],
    package_dir={'': 'src'},
)

setup(**d)
