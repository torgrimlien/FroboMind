from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d= generate_distutils_setup(
#    version='0.0.0',
    scripts=['src/waypoint_holder_node.py'],
    packages=['waypoint_holder'],
    package_dir={'': 'src'}
)
setup(**d)
