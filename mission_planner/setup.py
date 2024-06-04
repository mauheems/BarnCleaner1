from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mission_planner'],
    scripts=['scripts/mission_planner'],
    package_dir={'':'src'}
)

setup(**d)
