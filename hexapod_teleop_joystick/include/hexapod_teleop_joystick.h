
// ROS Hexapod Teleop Joystick Node
// Copyright (c) 2014, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#ifndef HEXAPOD_TELEOP_H_
#define HEXAPOD_TELEOP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/AccelStamped.h>

class HexapodTeleopJoystick
{
    public:
        HexapodTeleopJoystick( void );
        std_msgs::Bool state_;
        std_msgs::Bool imu_override_;
        geometry_msgs::AccelStamped body_scalar_;
        geometry_msgs::AccelStamped head_scalar_;
        geometry_msgs::Twist cmd_vel_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher base_scalar_pub_;
        ros::Publisher body_scalar_pub_;
        ros::Publisher head_scalar_pub_;
        ros::Publisher state_pub_;
        ros::Publisher imu_override_pub_;
        bool NON_TELEOP; // Shuts down cmd_vel broadcast

    private:
        void joyCallback( const sensor_msgs::Joy::ConstPtr &joy );
        ros::NodeHandle nh_;
        ros::Subscriber joy_sub_;
        int STANDUP_BUTTON, SITDOWN_BUTTON, BODY_ROTATION_BUTTON, FORWARD_BACKWARD_AXES, LEFT_RIGHT_AXES, YAW_ROTATION_AXES, PITCH_ROTATION_AXES;
        double MAX_METERS_PER_SEC, MAX_RADIANS_PER_SEC;
};

#endif // HEXAPOD_TELEOP_H_
