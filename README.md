

# ROS Hexapod Stack


## 1. Documentation

This is my implementation of a hexapod functioning in the ROS framework. Agnostic to either a 3dof or 4dof hexapod. It is still very much a work in progress and I am still actively developing it. 

Gait style is a simple sinusoidal tripod gait. I chose it due to its simplicity and smooth transitions between steps.

* Author: Kevin M. Ochs
* Contributor: Renée Love
* Contributor: Konstantinos Chatzilygeroudis
* Contributor: Kurt Eckhardt
* Contributor: Romain Reignier

## 2. Expected Hardware for mapping

* Primesense Sensor, Asus Xtion or Intel Realsense
* IMU (Current master branch uses a Phidgets 3/3/3 Spatial in launch files.)

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
sudo apt-get install ros-indigo-robot-localization
sudo apt-get install ros-indigo-rtabmap
sudo apt-get install ros-indigo-rtabmap-ros
sudo apt-get install ros-indigo-robot-state-publisher
sudo apt-get install ros-indigo-gazebo-ros-control
sudo apt-get install ros-indigo-navigation
sudo apt-get install ros-indigo-move_base
sudo apt-get install ros-indigo-navfn
sudo apt-get install ros-indigo-amcl
sudo apt-get install libusb-1.0-0-dev
```

**_Joystick_**


For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

https://help.ubuntu.com/community/Sixaxis

## 4. Nodes

**_hexapod_controller_**

This is the main node of the stack. It handles all control, gait, IK and servo communications with the legs. Minimal latency was required to keep the gait smooth and synced with odometry hence the reason they are all combined in this one node.

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist) Velocity command. 
     body_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the orientation of the body.
     head_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the pan and tilt of the optional turret.
     state (std_msgs::Bool) Bool array to record state of the hexapod. Standing up, sitting etc.
     imu/data (sensor_msgs::Imu) Used in optional auto body leveling on non level ground.
     
*Published Topics*

    sounds (hexapod_msgs::Sounds) Custom message to send sound cues to the optional sound package.
    joint_states (sensor_msgs::JointState) Joint states for rviz and such.
    odometry/calculated (nav_msgs::Odometry) Calculated odometry from the gait system in the package.
    twist (geometry_msgs::TwistWithCovarianceStamped) Twist message syncronized with the gait system. 
     

**_hexapod_bringup_**

This package has all the launch files. From simple locomotion only to full mapping and localization examples. 

**_hexapod_description_**

This package has all the param files. You will start with one of the param config files to describe your hexapod. It also has params for different telop controllers. The xacro and meshes also reside in this package.


**Example Launch Command**
```
roslaunch hexapod_bringup hexapod_full.launch config:=phantomX joy_mapping:=joystick_ds3
```
## 5. Install

```
git clone https://github.com/KevinOchs/hexapod_ros.git . 
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

## Videos 
------
_Click on picture for redirect to YouTube video._


Rviz recording of 3D mapping using RTABmap.

[![ScreenShot](http://img.youtube.com/vi/-3Ejgy1nFOg/0.jpg)]
(https://www.youtube.com/watch?v=-3Ejgy1nFOg)

Small video of Golem research platform and IMU testing.

[![ScreenShot](http://img.youtube.com/vi/IP-1HebkZnU/0.jpg)]
(https://www.youtube.com/watch?v=IP-1HebkZnU)

Renée Love's odometry test video using the phantomX.

[![ScreenShot](http://img.youtube.com/vi/VYBAM0MrvWI/0.jpg)]
(https://www.youtube.com/watch?v=VYBAM0MrvWI)


## Pictures

Rviz screenshot of point cloud and laserscan active.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/depthwithlaser.jpg)

2D room mapping in Rviz.
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/2d_slam.jpg)

Renée Love's adaptation of the Hexapod stack for Trossen's  [PhantomX](http://www.trossenrobotics.com/phantomx-ax-hexapod.aspx).
![ScreenShot](http://forums.trossenrobotics.com/gallery/files/1/2/6/6/9/screenshot_from_2015-04-22_20_23_15.png)

