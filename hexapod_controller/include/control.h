
// ROS Hexapod Locomotion Node
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


#ifndef CONTROL_H_
#define CONTROL_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <hexapod_msgs/RootJoint.h>
#include <hexapod_msgs/BodyJoint.h>
#include <hexapod_msgs/HeadJoint.h>
#include <hexapod_msgs/LegJoints.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FootPosition.h>
#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/State.h>

//==============================================================================
// Define class Control: This is the main structure of data that manipulates
// Golem.
//==============================================================================

class Control
{
    public:
        Control( void );
        void setHexActiveState( bool state );
        bool getHexActiveState( void );
        void setPrevHexActiveState( bool state );
        bool getPrevHexActiveState( void );
        hexapod_msgs::RootJoint root_;
        hexapod_msgs::BodyJoint body_;
        hexapod_msgs::HeadJoint head_;
        hexapod_msgs::LegsJoints legs_;
        hexapod_msgs::FeetPositions feet_;
        hexapod_msgs::State state_;
		sensor_msgs::Imu init_IMU_;
		sensor_msgs::Imu IMU_;
    private:
        bool hex_state_;      // Current loop state
        bool prev_hex_state_; // Previous loop state
		bool IMU_init_store_;
		double roll_init_;
		double pitch_init_;
        void rootCallback( const hexapod_msgs::RootJointConstPtr &root_msg );
        void bodyCallback( const hexapod_msgs::BodyJointConstPtr &body_msg );
        void headCallback( const hexapod_msgs::HeadJointConstPtr &head_msg );
        void stateCallback( const hexapod_msgs::StateConstPtr &state_msg );
        void IMUCallback( const sensor_msgs::ImuConstPtr &imu_msg );
        ros::NodeHandle nh_;
        ros::Subscriber root_sub_;
        ros::Subscriber body_sub_;
        ros::Subscriber head_sub_;
        ros::Subscriber state_sub_;
		ros::Subscriber IMU_sub_;
};

#endif // CONTROL_H_

