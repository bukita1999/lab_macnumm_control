from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['mecanum_control'],
    package_dir={'': 'src'},
    install_requires=['python-can']
)

setup(**setup_args)