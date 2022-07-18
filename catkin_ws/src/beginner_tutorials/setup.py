## ! DO NOT MANUALLY RUN THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['azure_services','speech_detection'],
    package_dir={'': 'src'}
)

setup(**d)
