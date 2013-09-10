from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/waypoint_holder_node.py','scripts/elevator_controller.py','scripts/measurement_dummy.py'],
  package_dir={'':'scripts'}
  )
setup(**d)
