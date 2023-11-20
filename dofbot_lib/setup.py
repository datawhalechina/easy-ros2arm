from setuptools import find_packages,setup
setup(
    name = 'Arm_Lib',
    version = '0.0.5',
    author='Others Team',
    packages = find_packages(),
)
# This package is used to build communication between Jetson and servo
# The package should be installed using following instruction,
# which can also be replaced by others so that create communication.

# cd py_install
# sudo python3 setup.py install
