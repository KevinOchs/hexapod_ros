
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

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/RPY.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Sounds.h>

//==============================================================================
// Define class Control: This is the main structure of data that manipulates
// the hexapod.
//==============================================================================

class Control
{
    public:
        Control( void );
        void setHexActiveState( bool state );
        bool getHexActiveState( void );
        void setPrevHexActiveState( bool state );
        bool getPrevHexActiveState( void );
        void publishJointStates( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::RPY &head, sensor_msgs::JointState *joint_state );
        void publishOdometry( const geometry_msgs::Twist &gait_vel );
        void publishTwist( const geometry_msgs::Twist &gait_vel );
        void partitionCmd_vel( geometry_msgs::Twist *cmd_vel );
        int MASTER_LOOP_RATE;  // Master loop rate
        sensor_msgs::JointState joint_state_;
        hexapod_msgs::Pose body_;    // Body link rotation
        hexapod_msgs::RPY head_;
        hexapod_msgs::LegsJoints legs_;
        hexapod_msgs::FeetPositions feet_;
        double STANDING_BODY_HEIGHT;
        geometry_msgs::Twist gait_vel_;
        geometry_msgs::Twist cmd_vel_;

    private:
        hexapod_msgs::Sounds sounds_; // Sound bool array
        std_msgs::Bool imu_override_; // Override body levelling for body manipulation
        bool imu_init_stored_; // Auto-levelling
        double imu_roll_lowpass_, imu_pitch_lowpass_, imu_yaw_lowpass_, imu_roll_init_, imu_pitch_init_; // Auto-levelling
        double MAX_BODY_ROLL_COMP, MAX_BODY_PITCH_COMP, COMPENSATE_INCREMENT, COMPENSATE_TO_WITHIN; // Auto-levelling yaml config
        double BODY_MAX_ROLL, BODY_MAX_PITCH, HEAD_MAX_YAW, HEAD_MAX_PITCH; // Mechanical limits
        double VELOCITY_DIVISION;
        double pose_x_;  // pose/odometry
        double pose_y_;  // pose/odometry
        double pose_th_; // pose/odometry
        int NUMBER_OF_LEGS;        // Number of legs
        int NUMBER_OF_HEAD_JOINTS; // Number of head segments
        int NUMBER_OF_LEG_JOINTS;  // Number of leg segments
        XmlRpc::XmlRpcValue SERVOS;
        std::vector<std::string> servo_map_key_;
        std::vector<std::string> servo_names_;
        std::vector<int> servo_orientation_;
        bool hex_state_;      // Current loop state
        bool prev_hex_state_; // Previous loop state
        ros::Time current_time_odometry_, last_time_odometry_, current_time_cmd_vel_, last_time_cmd_vel_;
        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::Twist cmd_vel_incoming_;

        // Topics we are subscribing
        ros::Subscriber cmd_vel_sub_;
        void cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg );
        ros::Subscriber body_scalar_sub_;
        void bodyCallback( const geometry_msgs::AccelStampedConstPtr &body_scalar_msg );
        ros::Subscriber head_scalar_sub_;
        void headCallback( const geometry_msgs::AccelStampedConstPtr &head_scalar_msg );
        ros::Subscriber state_sub_;
        void stateCallback( const std_msgs::BoolConstPtr &state_msg );
        ros::Subscriber imu_override_sub_;
        void imuOverrideCallback( const std_msgs::BoolConstPtr &imu_override_msg );
        ros::Subscriber imu_sub_;
        void imuCallback( const sensor_msgs::ImuConstPtr &imu_msg );

        // Topics we are publishing
        ros::Publisher sounds_pub_;
        ros::Publisher joint_state_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher twist_pub_;

        // Services we call
        ros::ServiceClient imu_calibrate_;
        std_srvs::Empty calibrate_;

        // Node Handle
        ros::NodeHandle nh_;
};

#endif // CONTROL_H_
