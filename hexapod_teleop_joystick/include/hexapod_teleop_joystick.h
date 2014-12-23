
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


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <hexapod_msgs/RootJoint.h>
#include <hexapod_msgs/BodyJoint.h>
#include <hexapod_msgs/HeadJoint.h>
#include <hexapod_msgs/State.h>

class HexapodTeleopJoystick
{
    public:
        HexapodTeleopJoystick( void );
        hexapod_msgs::RootJoint root_;
        hexapod_msgs::BodyJoint body_;
        hexapod_msgs::HeadJoint head_;
        hexapod_msgs::State state_;
        ros::Publisher root_pub_;
        ros::Publisher body_pub_;
        ros::Publisher head_pub_;
        ros::Publisher state_pub_;
    private:
        void joyCallback( const sensor_msgs::Joy::ConstPtr &joy );
        ros::NodeHandle nh_;
        ros::Subscriber joy_sub_;
};
