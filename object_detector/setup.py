from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["object_detector", "custom_msg_util"],
    scripts=["scripts/run_object_detector", "scripts/custom_msg_util"],
    package_dir={"": "src"},
)

setup(**d)
