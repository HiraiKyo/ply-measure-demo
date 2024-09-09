from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ply-measure-demo'],
    package_dir={'': 'src'},
    scripts=[]
)

setup(**d)