from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mecanum_control'],
    package_dir={'mecanum_control': 'src'},
    py_modules=['mecanum_controller', 'python_can_controller', 'motor_controller', 'trajectory_planner']
)

setup(**d)
