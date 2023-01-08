#! /usr/bin/python3
""" ROS Trimble BD990 Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nmea_socket_pub'],
    package_dir={'': 'src'}
)

setup(**d)
