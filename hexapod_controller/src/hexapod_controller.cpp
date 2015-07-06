
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


#include <ros/ros.h>
#include <control.h>
#include <gait.h>
#include <ik.h>
#include <servo_driver.h>

//=============================================================================
// Main
//=============================================================================

int main( int argc, char **argv )
{
    ros::init(argc, argv, "hexapod_locomotion");
    ROS_INFO("Hexapod locomotion node is running.");

    // Create class objects
    Control control;
    Gait gait;
    Ik ik;
    ServoDriver servoDriver;

    // Establish initial leg positions for default pose in robot publisher
    ik.calculateIK( control.feet_, control.body_, &control.legs_ );

    ros::AsyncSpinner spinner( 2 ); // Using 2 threads
    spinner.start();
    ros::Rate loop_rate( 1000 );  // 1000 hz
    while( ros::ok() )
    {
        // Start button on controller has been pressed stand up
        if( control.getHexActiveState() == true && control.getPrevHexActiveState() == false )
        {
            while( control.body_.position.z < control.STANDING_BODY_HEIGHT )
            {
                control.body_.position.z++;

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_ );
                servoDriver.transmitServoPositions( control.legs_, control.head_ );

            }
            control.setPrevHexActiveState( true );
            ROS_INFO("Hexapod standing up.");
        }

        // We are live and standing up
        if( control.getHexActiveState() == true && control.getPrevHexActiveState() == true )
        {
            // Gait Sequencer
            gait.gaitCycle( control.base_, &control.feet_ );

            // IK solver for legs and body orientation
            ik.calculateIK( control.feet_, control.body_, &control.legs_ );

            // Commit new positions and broadcast over USB2AX as well as jointStates
            control.publishJointStates( control.legs_, control.head_ );
            servoDriver.transmitServoPositions( control.legs_, control.head_ );

            // Set previous hex state of last loop so we know if we are shutting down on the next loop
            control.setPrevHexActiveState( true );
        }

        // Shutting down hex so let us do a gradual sit down and turn off torque
        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == true )
        {
            while( control.body_.position.z > 0 )
            {
                control.body_.position.z--;

                // Gait Sequencer called to make sure we are on all six feet
                gait.gaitCycle( control.base_, &control.feet_ );

                // IK solver for legs and body orientation
                ik.calculateIK( control.feet_, control.body_, &control.legs_ );

                // Commit new positions and broadcast over USB2AX as well as jointStates
                control.publishJointStates( control.legs_, control.head_ );
                servoDriver.transmitServoPositions( control.legs_, control.head_ );
            }

            // Release torque
            ros::Duration( 0.5 ).sleep();
            servoDriver.freeServos();
            ROS_INFO("Hexapod servos torque is now off.");

            // Locomotion is now shut off
            control.setPrevHexActiveState( false );
        }

        if( control.getHexActiveState() == false && control.getPrevHexActiveState() == false )
        {
            ros::Duration( 0.5 ).sleep();
            control.publishJointStates( control.legs_, control.head_ );
        }
        loop_rate.sleep();
    }
    return 0;
}
