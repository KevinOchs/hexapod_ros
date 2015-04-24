ROS Hexapod Stack
=================
This is a work in progress of my implementation of a hexapod functioning in the ROS framework. It is still very much a work in progress and I am still actively developing it. Its current capabilities are up to mapping its environment.

Dependencies
------------

```
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-diagnostic-updater
sudo apt-get install ros-indigo-xacro
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-depthimage-to-laserscan
sudo apt-get install ros-indigo-joystick-drivers
sudo apt-get install ros-indigo-imu-filter-madgwick
sudo apt-get install libusb-1.0-0-dev
```

_Joystick_
----------

For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

https://help.ubuntu.com/community/Sixaxis

Install
-------

```
git clone https://github.com/KevinOchs/ROS_hexapod.git
```

For PhantomX:

```
git checkout PhantomX
```
