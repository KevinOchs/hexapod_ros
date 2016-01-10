
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


#ifndef GAIT_H_
#define GAIT_H_

#include <cmath> // std::abs
#include <algorithm>
#include <ros/ros.h>
#include <hexapod_msgs/FeetPositions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

//=============================================================================
// Define structs and classes for gait system
//=============================================================================

class Gait
{
    public:
        Gait( void );
        void gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel );
    private:
        void cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet, geometry_msgs::Twist *gait_vel );
        geometry_msgs::Pose2D smooth_base_;
        ros::Time current_time_, last_time_;
        bool is_travelling_;      // True if the robot is moving, not just in a cycle
        bool in_cycle_;           // True if the robot is in a gait cycle
        int CYCLE_LENGTH;         // Number of steps in cycle
        int NUMBER_OF_LEGS;       // Leg order in cycle of the leg
        double LEG_LIFT_HEIGHT;   // Height of a leg cycle
        int cycle_period_;        // Current period in cycle
        std::vector<int> cycle_leg_number_; // Leg order in cycle of the leg
        int extra_gait_cycle_;    // Forcing some extra timed cycles
};

#endif // GAIT_H_
