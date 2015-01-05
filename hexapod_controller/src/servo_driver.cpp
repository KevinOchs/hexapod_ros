
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

#include <servo_driver.h>
static const double PI = atan(1.0)*4.0;
static const double RAD_TO_MX_RESOLUTION = ( MX_CENTER_VALUE*2 ) / ( PI*2 );

//=============================================================================
// Servo ID array ( Does not change )
//=============================================================================

static const int servo_id[SERVO_COUNT] =
{
    RR_COXA_ID,   RM_COXA_ID,   RF_COXA_ID,   LR_COXA_ID,   LM_COXA_ID,   LF_COXA_ID,
    RR_FEMUR_ID,  RM_FEMUR_ID,  RF_FEMUR_ID,  LR_FEMUR_ID,  LM_FEMUR_ID,  LF_FEMUR_ID,
    RR_TIBIA_ID,  RM_TIBIA_ID,  RF_TIBIA_ID,  LR_TIBIA_ID,  LM_TIBIA_ID,  LF_TIBIA_ID,
    RR_TARSUS_ID, RM_TARSUS_ID, RF_TARSUS_ID, LR_TARSUS_ID, LM_TARSUS_ID, LF_TARSUS_ID,
    HEAD_PAN_ID
};

//==============================================================================
//  Constructor: Open USB2AX and read current servo positions if they have power
// If servos are not on, no worries we read them again later just to be safe
//==============================================================================

ServoDriver::ServoDriver( void )
{
    int baudnum = 1;
    int deviceIndex = 0;

    if( dxl_initialize(deviceIndex, baudnum) == 0 )
    {
        ROS_ERROR("Servo Communication Failed!");
    }
    else
    {
        ROS_INFO( "Servo Communication Opened!" );
    }
    servos_free_ = true;

    ros::param::get("OFFSET_ANGLE", OFFSET_ANGLE );
}

//==============================================================================
// Convert angles to servo ticks each leg and head pan
//==============================================================================

void ServoDriver::convertAngles( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::HeadJoint &head )
{
    for( int leg_index = 0; leg_index <= 5; leg_index++ )
    {
        // Update Right Legs
        if( leg_index <= 2 )
        {
            goal_pos_[FIRST_COXA_ID   + leg_index] = MX_CENTER_VALUE + round( -legs.leg[leg_index].coxa * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_FEMUR_ID  + leg_index] = MX_CENTER_VALUE + round(  ( legs.leg[leg_index].femur - OFFSET_ANGLE ) * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_TIBIA_ID  + leg_index] = MX_CENTER_VALUE + round( -( legs.leg[leg_index].tibia - OFFSET_ANGLE ) * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_TARSUS_ID + leg_index] = MX_CENTER_VALUE + round(  ( legs.leg[leg_index].tarsus - OFFSET_ANGLE*2 ) * RAD_TO_MX_RESOLUTION );
        }
        else
        // Update Left Legs
        {
            goal_pos_[FIRST_COXA_ID   + leg_index] = MX_CENTER_VALUE + round(  legs.leg[leg_index].coxa * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_FEMUR_ID  + leg_index] = MX_CENTER_VALUE + round( -( legs.leg[leg_index].femur - OFFSET_ANGLE ) * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_TIBIA_ID  + leg_index] = MX_CENTER_VALUE + round(  ( legs.leg[leg_index].tibia - OFFSET_ANGLE ) * RAD_TO_MX_RESOLUTION );
            goal_pos_[FIRST_TARSUS_ID + leg_index] = MX_CENTER_VALUE + round( -( legs.leg[leg_index].tarsus - OFFSET_ANGLE*2 ) * RAD_TO_MX_RESOLUTION );
        }
    }
    goal_pos_[24] = MX_CENTER_VALUE + round( head.yaw * RAD_TO_MX_RESOLUTION );
}

//==============================================================================
// Turn torque on and read current positions
//==============================================================================

void ServoDriver::makeSureServosAreOn( void )
{
    if( !servos_free_ )
    {
        // Servos are on so return
        return;
    }

    // Initialize current position as cur since values would be 0 for all servos ( Possibly servos are off till now )
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        cur_pos_[i] = dxl_read_word( servo_id[i], MX_PRESENT_POSITION_L );
        ros::Duration( 0.01 ).sleep();
    }

    // Turn torque on
    dxl_write_word( 254, MX_TORQUE_ENABLE, 1 );
    servos_free_ = false;
}

//==============================================================================
// Updates the positions of the servos and sends USB2AX broadcast packet
//==============================================================================

void ServoDriver::transmitServoPositions( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::HeadJoint &head )
{
    convertAngles( legs, head );

    makeSureServosAreOn();

    int interpolating = 0;
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        // If any of these differ we need to indicate a new packet needs to be sent
        if( cur_pos_[i] != goal_pos_[i] )
        {
            interpolating++;
        }
    }

    // If nothing moved we abort no need to send packet with same positions
    if( interpolating != 0 )
    {
        int complete[SERVO_COUNT];
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            if( cur_pos_[i] == goal_pos_[i] )
            {
                // Nothing is moving on this particular servo
                pose_steps_[i] = 0;
                write_pos_[i] = goal_pos_[i];
                complete[i] = 1;
            }
            else
            {
                pose_steps_[i] = 1;
                write_pos_[i] = cur_pos_[i];
                complete[i] = 0;
            }
        }

        bool finished = false;
        ros::Rate loop_rate( 900 ); // 900 Hz loop
        while( finished == false )
        {
            // Prepare packet for broadcast
            dxl_set_txpacket_id( 254 );
            dxl_set_txpacket_instruction( 131 );
            dxl_set_txpacket_length( 5 * SERVO_COUNT + 4 );
            dxl_set_txpacket_parameter( 0, MX_GOAL_POSITION_L );
            dxl_set_txpacket_parameter( 1, 4 );

            int total_complete = 0;
            for( int i = 0; i < SERVO_COUNT; i++ )
            {
                if( pose_steps_[i] == 1 && complete[i] != 1 )
                {
                    if( cur_pos_[i] < goal_pos_[i] )
                    {
                        write_pos_[i] = write_pos_[i] + pose_steps_[i];

                        if( write_pos_[i] >= goal_pos_[i] )
                        {
                            write_pos_[i] = goal_pos_[i];
                            complete[i] = 1;
                        }
                    }

                    if( cur_pos_[i] > goal_pos_[i] )
                    {
                        write_pos_[i] = write_pos_[i] - pose_steps_[i];

                        if( write_pos_[i] <= goal_pos_[i] )
                        {
                            write_pos_[i] = goal_pos_[i];
                            complete[i] = 1;
                        }
                    }
                }

                // Tally up which servos are at their goal position
                if( complete[i] == 1 )
                {
                    total_complete++;
                }

                // Complete sync_write packet for broadcast
                dxl_set_txpacket_parameter( 2 + 5 * i, servo_id[i] );
                dxl_set_txpacket_parameter( 2 + 5 * i + 1, dxl_get_lowbyte( write_pos_[i] ) );
                dxl_set_txpacket_parameter( 2 + 5 * i + 2, dxl_get_highbyte( write_pos_[i] ) );
            }

            // Broadcast packet over USB2AX
            dxl_txrx_packet();

            // Since we loop until all servos are finished we check here if complete to stop loop
            if( total_complete == 25 )
            {
                finished = true;
            }
            loop_rate.sleep();
        }

        // Store write pose as current pose (goal) since we are now done
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            cur_pos_[i] = write_pos_[i];
        }
    }
}

//==============================================================================
// Turn torque off to all servos
//==============================================================================

void ServoDriver::freeServos( void )
{
    // Turn off torque
    dxl_write_word( 254, MX_TORQUE_ENABLE, 0 );
    servos_free_ = true;
}

