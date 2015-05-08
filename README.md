

# ROS Hexapod Stack


## 1. Documentation

This is my implementation of a hexapod functioning in the ROS framework. It is still very much a work in progress and I am still actively developing it. Its current capabilities are up to 2D mapping its environment. Full navigation stack compliance will be pushed up in mid-May 2015.

Gait style is a simple sinusoidal tripod gait. I chose it due to its simplicity and smooth transitions between steps.

* Author: Kevin M. Ochs
* Contributor: Renée Love

## 2. Expected Hardware

* Primesense Sensor or Asus Xtion
* IMU (Current master branch uses a Phidgets 3/3/3 Spatial

## 3. Dependencies

```
sudo apt-get install git
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-diagnostic-updater
sudo apt-get install ros-indigo-xacro
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-depthimage-to-laserscan
sudo apt-get install ros-indigo-joystick-drivers
sudo apt-get install ros-indigo-imu-filter-madgwick
sudo apt-get install ros-indigo-gmapping
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libfovis
```

**_Joystick_**


For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

https://help.ubuntu.com/community/Sixaxis

## 4. Install

```
git clone https://github.com/KevinOchs/ROS_hexapod.git
```

For PhantomX branch created by Renée Love:

```
git checkout PhantomX
```

For Raspberry Pi2 please add these compiler optimizations after first build.
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7
```

For ODROID XU3 please add these compiler optimizations after first build.
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -pipe -march=armv7-a -mcpu=cortex-a9 -mfloat-abi=hard
```


## Pictures

Rviz screenshot of point cloud and laserscan active.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/depthwithlaser.jpg)

2D room mapping in Rviz.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/2d_slam.jpg)

Renée Love's adaptation of the Hexapod stack for Trossen's  [PhantomX](http://www.trossenrobotics.com/phantomx-ax-hexapod.aspx).
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/1/2/6/6/9/screenshot_from_2015-04-22_20_23_15.png)


Videos 
------
_Click on picture for redirect to YouTube video._

Small video of Golem research platform and IMU testing. 
[![ScreenShot](http://img.youtube.com/vi/IP-1HebkZnU/0.jpg)](https://www.youtube.com/watch?v=IP-1HebkZnU)

Renée Love's odometry test video using the phantomX.
[![ScreenShot](http://img.youtube.com/vi/VYBAM0MrvWI/0.jpg)](https://www.youtube.com/watch?v=VYBAM0MrvWI)
