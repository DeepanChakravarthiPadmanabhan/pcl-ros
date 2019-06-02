from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['table_setup_ros'],
    package_dir={'table_setup_ros' : 'ros/src/table_setup_ros'}
)

setup(**d)
