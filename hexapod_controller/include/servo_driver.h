
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
#include <hexapod_msgs/HeadJoint.h>
#include <hexapod_msgs/LegsJoints.h>

//====================================================================
// Number servos in system
//====================================================================

#define SERVO_COUNT 25

//====================================================================
// Define Servo ID's that go with each joint
//====================================================================

#define RR_COXA_ID   1  // Rear Right leg Coxa
#define RR_FEMUR_ID  2  // Rear Right leg Femur
#define RR_TIBIA_ID  3  // Rear Right leg Tibia
#define RR_TARSUS_ID 4  // Rear Right leg Tarsus

#define RM_COXA_ID   5  // Middle Right leg Coxa
#define RM_FEMUR_ID  6  // Middle Right leg Femur
#define RM_TIBIA_ID  7  // Middle Right leg Tibia
#define RM_TARSUS_ID 8  // Middle Right leg Tarsus

#define RF_COXA_ID   9  // Front Right leg Coxa
#define RF_FEMUR_ID  10 // Front Right leg Femur
#define RF_TIBIA_ID  11 // Front Right leg Tibia
#define RF_TARSUS_ID 12 // Front Right leg Tarsus

#define LR_COXA_ID   13 // Rear Left leg Coxa
#define LR_FEMUR_ID  14 // Rear Left leg Femur
#define LR_TIBIA_ID  15 // Rear Left leg Tibia
#define LR_TARSUS_ID 16 // Rear Left leg Tarsus

#define LM_COXA_ID   17 // Middle Left leg Coxa
#define LM_FEMUR_ID  18 // Middle Left leg Femur
#define LM_TIBIA_ID  19 // Middle Left leg Tibia
#define LM_TARSUS_ID 20 // Middle Left leg Tarsus

#define LF_COXA_ID   21 // Front Left leg Coxa
#define LF_FEMUR_ID  22 // Front Left leg Femur
#define LF_TIBIA_ID  23 // Front Left leg Tibia
#define LF_TARSUS_ID 24 // Front Left leg Tarsus

#define HEAD_PAN_ID  25 // Head Pan


//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================

class ServoDriver
{
    public:
        ServoDriver( void );
        void transmitServoPositions( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::HeadJoint &head );
        void freeServos( void );
    private:
        void convertAngles( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::HeadJoint &head );
        void makeSureServosAreOn( void );
        std::vector<int> cur_pos_;
        std::vector<int> goal_pos_;
        std::vector<int> pose_steps_;
        std::vector<int> write_pos_;
        bool servos_free_;
        double OFFSET_ANGLE;
        int FIRST_COXA_ID, FIRST_FEMUR_ID, FIRST_TIBIA_ID, FIRST_TARSUS_ID;
        int NUMBER_OF_LEGS;       // Number of legs
};

#endif

