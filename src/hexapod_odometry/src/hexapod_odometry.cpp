
// ROS Hexapod Odometry Node
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


#include <hexapod_odometry.h>

//==============================================================================
// Constructor
//==============================================================================

HexapodOdometry::HexapodOdometry( void )
{
    base_sub_ = nh_.subscribe<hexapod_msgs::RootJoint>( "base", 50, &HexapodOdometry::odometryCallback, this );
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
}

//==============================================================================
// Reading base topic
//==============================================================================


void HexapodOdometry::odometryCallback( const hexapod_msgs::RootJointConstPtr &base_msg )
{
    vx = -base_msg->x / 1000; // FLIPPING SO IT WORKS SO NEED TO FIX THE MATH
    vy = base_msg->y / 1000;
    vth = -base_msg->yaw * 3.75; // FLIPPING SO IT WORKS SO NEED TO FIX THE MATH
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_odometry");

    HexapodOdometry hexapodOdometry;
    tf::TransformBroadcaster odom_broadcaster;
    ros::AsyncSpinner spinner(1); // Using 1 thread
    spinner.start();

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate( 1000 ); // 1000 hz
    while( ros::ok() )
    {
        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = ( current_time - last_time ).toSec();
        double delta_x = ( hexapodOdometry.vx * cos( th ) - hexapodOdometry.vy * sin( th ) ) * dt;
        double delta_y = ( hexapodOdometry.vx * sin( th ) + hexapodOdometry.vy * cos( th ) ) * dt;
        double delta_th = hexapodOdometry.vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( th );

        // first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_combined";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform
        // odom_broadcaster.sendTransform( odom_trans ); // ONLY NEEDED FOR DEBUGGING!!! DON'T USE WITH EKF

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom_combined";
        odom.child_frame_id = "base_footprint";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.pose.covariance[0] = 0.0001;  // x
        odom.pose.covariance[7] = 0.0001;  // y
        odom.pose.covariance[14] = 5;      // z
        odom.pose.covariance[21] = 1;      // rot x
        odom.pose.covariance[28] = 1;      // rot y
        odom.pose.covariance[35] = 1;      // rot z

        //set the velocity
        odom.twist.twist.linear.x = hexapodOdometry.vx;
        odom.twist.twist.linear.y = hexapodOdometry.vy;
        odom.twist.twist.angular.z = hexapodOdometry.vth;
        odom.twist.covariance = odom.pose.covariance;

        hexapodOdometry.odom_pub_.publish( odom );
        last_time = current_time;
        loop_rate.sleep();
    }
}
