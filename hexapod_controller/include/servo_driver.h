
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


#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <cmath>
#include <ros/ros.h>
#include <dynamixel.h>
#include <mx.h>
#include <sensor_msgs/JointState.h>

//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================

class ServoDriver
{
    public:
        ServoDriver( void );
        void transmitServoPositions( const sensor_msgs::JointState &joint_state );
        void prepareServoSettings( const sensor_msgs::JointState &joint_state );
        void freeServos( void );
    private:
        void convertAngles( const sensor_msgs::JointState &joint_state );
        void makeSureServosAreOn( const sensor_msgs::JointState &joint_state );
        std::vector<int> cur_pos_;
        std::vector<int> goal_pos_;
        std::vector<int> pose_steps_;
        std::vector<int> write_pos_;
        std::vector<double> OFFSET;
        std::vector<int> ID;
        std::vector<int> TICKS;
        std::vector<double> RAD_TO_SERVO_RESOLUTION;
        std::vector<double> MAX_RADIANS;
        bool servos_free_;
        double OFFSET_ANGLE;
        int FIRST_COXA_INDEX, FIRST_FEMUR_INDEX, FIRST_TIBIA_INDEX, FIRST_TARSUS_INDEX;
        int SERVO_COUNT;
};

#endif
