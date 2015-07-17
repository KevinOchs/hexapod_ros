
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
    state_.data = false;
    imu_override_.data = false;
    ros::param::get( "MAX_METERS_PER_SEC", MAX_METERS_PER_SEC );
    ros::param::get( "MAX_RADIANS_PER_SEC", MAX_RADIANS_PER_SEC );
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 5, &HexapodTeleopJoystick::joyCallback, this);
    base_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("base_scalar", 100);
    body_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("body_scalar", 100);
    head_scalar_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("head_scalar", 100);
    state_pub_ = nh_.advertise<std_msgs::Bool>("state", 100);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    imu_override_pub_ = nh_.advertise<std_msgs::Bool>("imu_override", 100);
}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void HexapodTeleopJoystick::joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    ros::Time current_time = ros::Time::now();
    if ( joy->buttons[3] == 1 )
    {
        if ( state_.data == false)
        {
            state_.data = true;
        }
    }

    if ( joy->buttons[0] == 1 )
    {
        if ( state_.data == true)
        {
            state_.data = false;
        }
    }

    // Body shift L1 Button for testing
    if ( joy->buttons[8] == 1 )
    {
        imu_override_.data = true;
        body_scalar_.header.stamp = current_time;
        body_scalar_.accel.angular.x = -joy->axes[0];
        body_scalar_.accel.angular.y = -joy->axes[1];
        head_scalar_.header.stamp = current_time;
        head_scalar_.accel.angular.z = joy->axes[2];
    }
    else
    {
        imu_override_.data = false;
    }

    // Travelling
    if ( joy->buttons[8] != 1 )
    {
        base_scalar_.header.stamp = current_time;
        base_scalar_.accel.linear.x = -joy->axes[1];
        base_scalar_.accel.linear.y = joy->axes[0];
        base_scalar_.accel.angular.z = -joy->axes[2];
        cmd_vel_.linear.x = ( joy->axes[1] * MAX_METERS_PER_SEC );
        cmd_vel_.linear.y = ( joy->axes[0] * MAX_METERS_PER_SEC );
        cmd_vel_.angular.z = ( joy->axes[2] * MAX_RADIANS_PER_SEC );
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_teleop_joystick");
    HexapodTeleopJoystick hexapodTeleopJoystick;

    ros::AsyncSpinner spinner(1); // Using 1 threads
    spinner.start();

    ros::Rate loop_rate( 250 ); // 250 hz
    while ( ros::ok() )
    {
        hexapodTeleopJoystick.base_scalar_pub_.publish( hexapodTeleopJoystick.base_scalar_ );
        hexapodTeleopJoystick.body_scalar_pub_.publish( hexapodTeleopJoystick.body_scalar_ );
        hexapodTeleopJoystick.head_scalar_pub_.publish( hexapodTeleopJoystick.head_scalar_ );
        hexapodTeleopJoystick.state_pub_.publish( hexapodTeleopJoystick.state_ );
        hexapodTeleopJoystick.cmd_vel_pub_.publish( hexapodTeleopJoystick.cmd_vel_ );
        hexapodTeleopJoystick.imu_override_pub_.publish( hexapodTeleopJoystick.imu_override_ );
        loop_rate.sleep();
    }
}
