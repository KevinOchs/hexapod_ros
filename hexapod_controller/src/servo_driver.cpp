// ROS Hexapod Controller Node
// Copyright (c) 2016, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the Kevin Ochs nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL KEVIN OCHS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs


#include <servo_driver.h>

//==============================================================================
//  Constructor: Open USB2AX and get parameters
// If servos are not on, no worries we read them later just to be safe
//==============================================================================

ServoDriver::ServoDriver( void )
{
    // Open port
    if ( portHandler->openPort() )
    {
        ROS_INFO("Succeeded to open the port!");
            // Set port baudrate
            if ( portHandler->setBaudRate(BAUDRATE) ) ROS_INFO("Succeeded to change the baudrate!");
            else ROS_WARN("Failed to change the baudrate!");
            portOpenSuccess = true;
    }
    else ROS_WARN("Failed to open the USB port!, Ignore if using Rviz or Gazbebo");

    // Stating servos do not have torque applied
    servos_free_ = true;
    ros::param::get( "TORQUE_ENABLE", TORQUE_ENABLE );
    ros::param::get( "PRESENT_POSITION_L", PRESENT_POSITION_L );
    ros::param::get( "GOAL_POSITION_L", GOAL_POSITION_L );
    ros::param::get( "SERVOS", SERVOS );
    ros::param::get( "INTERPOLATION_LOOP_RATE", INTERPOLATION_LOOP_RATE );
    for( XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++ )
    {
        servo_map_key_.push_back( it->first );
    }

    SERVO_COUNT = servo_map_key_.size();
    OFFSET.resize( SERVO_COUNT );
    ID.resize( SERVO_COUNT );
    TICKS.resize( SERVO_COUNT );
    CENTER.resize( SERVO_COUNT );
    MAX_RADIANS.resize( SERVO_COUNT );
    RAD_TO_SERVO_RESOLUTION.resize( SERVO_COUNT );
    servo_orientation_.resize( SERVO_COUNT );
    cur_pos_.resize( SERVO_COUNT );
    goal_pos_.resize( SERVO_COUNT );
    write_pos_.resize( SERVO_COUNT );
    pose_steps_.resize( SERVO_COUNT );
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/offset", OFFSET[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/id", ID[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/ticks", TICKS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/center", CENTER[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/max_radians", MAX_RADIANS[i] );
        ros::param::get( "SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/sign", servo_orientation_[i] );
        RAD_TO_SERVO_RESOLUTION[i] = TICKS[i] / MAX_RADIANS[i];
        // Fill vector containers with default value
        cur_pos_[i] = CENTER[i];
        goal_pos_[i] = CENTER[i];
        write_pos_[i] = CENTER[i];
        pose_steps_[i] = 1;
    }
}

//==============================================================================
// Destructor: Turn off the torque of the servos then close the serial port
//==============================================================================

ServoDriver::~ServoDriver( void )
{
    freeServos();
    portHandler->closePort();
}

//==============================================================================
// Convert angles to servo resolution each leg and head pan
//==============================================================================

void ServoDriver::convertAngles( const sensor_msgs::JointState &joint_state )
{
    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        goal_pos_[i] = CENTER[i] + round( ( joint_state.position[i] - ( servo_orientation_[i] * OFFSET[i] ) ) * RAD_TO_SERVO_RESOLUTION[i] );
    }
}

//==============================================================================
// Turn torque on and read current positions
//==============================================================================

void ServoDriver::makeSureServosAreOn( const sensor_msgs::JointState &joint_state )
{
    if( !servos_free_ )
    {
        // Servos are on so return
        return;
    }
    else
    {
        // Initialize current position as cur since values would be 0 for all servos ( Possibly servos are off till now )
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            // Read present position
            if( packetHandler->read2ByteTxRx(portHandler, ID[i], PRESENT_POSITION_L, &currentPos, &dxl_error) == COMM_SUCCESS )
            {
                cur_pos_[i] = currentPos;
                //ROS_INFO("[ID:%02d]  PresPos:%02d", ID[i], cur_pos_[i]);
            }
            else
            {
                if( portOpenSuccess ) ROS_WARN("Read error on [ID:%02d]", ID[i]);
            }
        }
        ros::Duration( 0.1 ).sleep();
        // Turn torque on
        for( int i = 0; i < SERVO_COUNT; i++ ){
            if( packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE, TORQUE_ON, &dxl_error) != COMM_SUCCESS && portOpenSuccess )
            {
                ROS_WARN("TURN TORQUE ON SERVO FAILED [ID:%02d]", ID[i]);
                torque_on = false;
            }
        }
        if( torque_on )
        {
            ROS_INFO("Hexapod servos torque is now ON.");
            servos_free_ = false;
        }
    }
}

//==============================================================================
// Updates the positions of the servos and sends USB2AX broadcast packet
//==============================================================================

void ServoDriver::transmitServoPositions( const sensor_msgs::JointState &joint_state )
{
    dynamixel::GroupSyncWrite groupSyncWrite( portHandler, packetHandler, GOAL_POSITION_L, LEN_GOAL_POSITION );
    convertAngles( joint_state ); // Convert angles to servo resolution
    makeSureServosAreOn( joint_state );

    int interpolating = 0;
    int complete[SERVO_COUNT];

    for( int i = 0; i < SERVO_COUNT; i++ )
    {
        // If any of these differ we need to indicate a new packet needs to be sent
        if( cur_pos_[i] != goal_pos_[i] )
        {
            interpolating++;
            pose_steps_[i] = 1;
            write_pos_[i] = cur_pos_[i];
            complete[i] = 0;
        }
        else
        {
            // Nothing is moving on this particular servo
            pose_steps_[i] = 0;
            write_pos_[i] = goal_pos_[i];
            complete[i] = 1;
        }
    }

    ros::Rate loop_rate( INTERPOLATION_LOOP_RATE ); // 900 Hz loop
    // If nothing moved we abort no need to send packet with same positions
    if( interpolating != 0 )
    {
        while( interpolating != 0 )
        {
            // Prepare packet for broadcast
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
                            interpolating--;
                        }
                    }

                    if( cur_pos_[i] > goal_pos_[i] )
                    {
                        write_pos_[i] = write_pos_[i] - pose_steps_[i];

                        if( write_pos_[i] <= goal_pos_[i] )
                        {
                            write_pos_[i] = goal_pos_[i];
                            complete[i] = 1;
                            interpolating--;
                        }
                    }
                }
                // Complete sync_write packet for broadcast
                param_goal_position[0] = DXL_LOBYTE(write_pos_[i]);
                param_goal_position[1] = DXL_HIBYTE(write_pos_[i]);
                if( !groupSyncWrite.addParam(ID[i], param_goal_position) && portOpenSuccess )
                {
                    ROS_WARN("Goal position param write failed on [ID:%02d]", ID[i]);
                    writeParamSuccess = false;
                }

            }
            // Broadcast packet over U2D2
            if( writeParamSuccess )
            {
                if( groupSyncWrite.txPacket() != COMM_SUCCESS && portOpenSuccess ) ROS_WARN("Position write not successfull!!");
            }
            groupSyncWrite.clearParam();
            loop_rate.sleep();
        }
        // Store write pose as current pose (goal) since we are now done
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            cur_pos_[i] = write_pos_[i];
        }
    }
    loop_rate.sleep();
}

//==============================================================================
// Turn torque off to all servos
//==============================================================================

void ServoDriver::freeServos( void )
{
    // Turn off torque
        for( int i = 0; i < SERVO_COUNT; i++ )
        {
            if( packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE, TORQUE_OFF, &dxl_error) != COMM_SUCCESS && portOpenSuccess )
            {
                ROS_WARN("TURN TORQUE OFF FAILED ON SERVO [ID:%02d]", ID[i]);
                torque_off = false;
            }
        }
        if( torque_off )
        {
            ROS_INFO("Hexapod servos torque is now OFF.");
            servos_free_ = true;
        }
}
