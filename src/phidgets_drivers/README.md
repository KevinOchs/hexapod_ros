Phidgets drivers for ROS Groovy/Hydro
=============================================

Overview
---------------------------------------------

Drivers for the Phidgets devices. This Catkin metapackage includes:

 * `phidgets_api`: a package which downloads and builds the Phidgets C API from
   phidgets.com (as an external project). It also implements a C++ wrapper
   for the C API, providing a base Phidget class and various inherited classes
   for different phidget devices.

 * Two packages exposing the functionality of specific phidgets devices using
   the ROS API: `phidgets_imu` and `phidgets_ir`.


Installing
---------------------------------------------

### From source ###

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

Download the metapackage from the github repository (<ros_distro> may be `groovy` or `hydro`):

    git clone -b <ros_distro> https://github.com/ccny-ros-pkg/phidgets_drivers.git

Install dependencies using rosdep:

    rosdep install phidgets_drivers

Alternatively, if rosdep does not work, install the following packages:

    sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make

### Udev rules setup: ###

Make sure your catkin workspace has been successfully compiled.
To set up the udev rules for the Phidgets USB devices, run the following commands:

    cd ~/catkin_ws
    sh src/phidgets_drivers/phidgets_api/share/setup-udev.sh

You will be prompted to type in your password.


For documentation regarding nodes, topics, etc:
---------------------------------------------

http://ros.org/wiki/phidgets_drivers
