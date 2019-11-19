#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

D = generate_distutils_setup(
    packages=[
        'rll_move_client',
    ],
    package_dir={'': 'src'}
)

setup(**D)
