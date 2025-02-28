#!/usr/bin/env python

from setuptools import find_packages
from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={
        '': 'src'
    },
    scripts=['scripts/main.py', 'scripts/react.py'],
)

setup(**setup_args)