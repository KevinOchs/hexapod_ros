
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


#include <hexapod_teleop_joystick.h>

//==============================================================================
// Constructor
//==============================================================================

HexapodTeleopJoystick::HexapodTeleopJoystick( void )
{
    base_.y = 0.0;
    base_.x = 0.0;
    base_.yaw = 0.0;
    body_.y = 0.0;
    body_.z = 0.0;
    body_.x = 0.0;
    body_.pitch = 0.0;
    body_.yaw = 0.0;
    body_.roll = 0.0;
    head_.yaw = 0.0;
    state_.active = false;
    imu_override_.active = false;
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 5, &HexapodTeleopJoystick::joyCallback, this);
    base_pub_ = nh_.advertise<hexapod_msgs::RootJoint>("base", 100);
    body_pub_ = nh_.advertise<hexapod_msgs::BodyJoint>("body", 100);
    head_pub_ = nh_.advertise<hexapod_msgs::HeadJoint>("head", 100);
    state_pub_ = nh_.advertise<hexapod_msgs::State>("state", 100);
    imu_override_pub_ = nh_.advertise<hexapod_msgs::State>("imu_override", 100);
}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void HexapodTeleopJoystick::joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    if ( joy->buttons[3] == 1 )
    {
        if ( state_.active == false)
        {
            state_.active = true;
        }
    }

    if ( joy->buttons[0] == 1 )
    {
        if ( state_.active == true)
        {
            state_.active = false;
        }
    }

    // Body shift L1 Button for testing
    if ( joy->buttons[8] == 1 )
    {
        imu_override_.active = true;
        body_.pitch = -joy->axes[1] * 0.13962634; // 8 degrees max
        body_.roll = -joy->axes[0] * 0.13962634; // 8 degrees max
        head_.yaw = joy->axes[2] * 0.27925268; // 16 degrees max
    }
    else
    {
        imu_override_.active = false;
    }

    // Travelling ( 8cm/s )
    if ( joy->buttons[8] != 1 )
    {
        base_.x = ( -joy->axes[1] * 40.0 ) * 0.05 + ( base_.x * ( 1.0 - 0.05 ) ); // 40 mm max
        base_.y = ( joy->axes[0] * 40.0 ) * 0.05 + ( base_.y * ( 1.0 - 0.05 ) ); // 40 mm max
        base_.yaw = ( -joy->axes[2] * 0.13962634 ) * 0.5 + ( base_.yaw * ( 1.0 - 0.5 ) ); // 8 degrees max
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_teleop_joystick");
    HexapodTeleopJoystick hexapodTeleopJoystick;

    ros::AsyncSpinner spinner(1); // Using 4 threads
    spinner.start();

    ros::Rate loop_rate( 2000 ); // 1000 hz
    while ( ros::ok() )
    {
        hexapodTeleopJoystick.base_pub_.publish( hexapodTeleopJoystick.base_ );
        hexapodTeleopJoystick.body_pub_.publish( hexapodTeleopJoystick.body_ );
        hexapodTeleopJoystick.head_pub_.publish( hexapodTeleopJoystick.head_ );
        hexapodTeleopJoystick.state_pub_.publish( hexapodTeleopJoystick.state_ );
        hexapodTeleopJoystick.imu_override_pub_.publish( hexapodTeleopJoystick.imu_override_ );
        loop_rate.sleep();
    }

}
