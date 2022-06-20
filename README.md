# Mirte Python API

This package provides the API for the Mirte Robot. Please read the Mirte documentation.

# IMPORTANT: Mirte Phone API

This package was expanded with a Phone api, for interacting with the components made available through mirtesensorlib

New functionality can be found in mirte_robot/phone.py

## Running the tests

For the new phone functionality, a number of unit tests is available.

Running these tests is a little tricky, as it requires ROS specific packages to work.
Therefore, ROS must be installed on the local device to run the tests.
Also, python3 must be installed on the device.

### Windows

For windows, please see this guide for installing ROS: http://wiki.ros.org/Installation/Windows

Once ROS has been installed, open the ROS Command Window shortcut from the guide, and run:

$ roscore

Afterwards open a new ROS Command Window, and run:

$ python3 -m unittest discover c:\path\to\repository

This should start the tests.

### Linux with ROS installation

For any linux system running ROS, running the following command should anywhere be sufficient to run the tests

$ python3 -m unittest discover /path/to/repository

Once again, do make sure ros master is running.

