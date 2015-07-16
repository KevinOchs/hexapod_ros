
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

#include <control.h>
static const double PI = atan(1.0)*4.0;
//==============================================================================
// Constructor
//==============================================================================

Control::Control( void )
{
    ros::param::get( "LEG_ORDER", JOINT_SUFFIX );
    ros::param::get( "LEG_SEGMENT_NAMES", LEG_SEGMENT_NAMES );
    ros::param::get( "HEAD_SEGMENT_NAMES", HEAD_SEGMENT_NAMES );
    ros::param::get( "BODY_MAX_ROLL", BODY_MAX_ROLL );
    ros::param::get( "BODY_MAX_PITCH", BODY_MAX_PITCH );
    ros::param::get( "HEAD_MAX_PAN", HEAD_MAX_PAN );
    ros::param::get( "CYCLE_MAX_TRAVEL", CYCLE_MAX_TRAVEL );
    ros::param::get( "CYCLE_MAX_YAW", CYCLE_MAX_YAW );
    ros::param::get( "STANDING_BODY_HEIGHT", STANDING_BODY_HEIGHT );
    ros::param::get( "SERVOS", SERVOS );
    for( XmlRpc::XmlRpcValue::iterator it = SERVOS.begin(); it != SERVOS.end(); it++ )
    {
        servo_map_key_.push_back(it->first);
    }
    servo_names_.resize(servo_map_key_.size());
    servo_orientation_.resize(servo_map_key_.size());
    for( int i = 0; i < servo_map_key_.size(); i++ )
    {
        ros::param::get( ("/SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/name"), servo_names_[i] );
        ros::param::get( ("/SERVOS/" + static_cast<std::string>( servo_map_key_[i] ) + "/sign"), servo_orientation_[i] );
    }
    prev_hex_state_ = false;
    hex_state_ = false;
    imu_init_stored_ = false;
    imu_override_.data = false;
    imu_roll_lowpass_ = 0.0;
    imu_pitch_lowpass_ = 0.0;
    imu_yaw_lowpass_ = 0.0;
    imu_roll_init_ = 0.0;
    imu_pitch_init_ = 0.0;
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
    base_.y = 0.0;
    base_.x = 0.0;
    base_.theta = 0.0;
    body_.position.y = 0.0;
    body_.position.z = 0.0;
    body_.position.x = 0.0;
    body_.orientation.pitch = 0.0;
    body_.orientation.yaw = 0.0;
    body_.orientation.roll = 0.0;
    head_.yaw = 0.0;
    NUMBER_OF_LEGS = JOINT_SUFFIX.size();
    NUMBER_OF_LEG_JOINTS = NUMBER_OF_LEGS * LEG_SEGMENT_NAMES.size();
    NUMBER_OF_HEAD_JOINTS = HEAD_SEGMENT_NAMES.size();
    NUMBER_OF_JOINTS = servo_map_key_.size();
    joint_state_.name.resize( NUMBER_OF_JOINTS );
    joint_state_.position.resize( NUMBER_OF_JOINTS );
    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        feet_.foot[leg_index].position.x = 0.0;
        feet_.foot[leg_index].position.y = 0.0;
        feet_.foot[leg_index].position.z = 0.0;
        feet_.foot[leg_index].orientation.yaw = 0.0;
        legs_.leg[leg_index].coxa = 0.0;
        legs_.leg[leg_index].femur = 0.0;
        legs_.leg[leg_index].tibia = 0.0;
        legs_.leg[leg_index].tarsus = 0.0;
    }
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "cmd_vel", 1, &Control::cmd_velCallback, this );
    base_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>( "base_scalar", 1, &Control::baseCallback, this );
    body_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>( "body_scalar", 1, &Control::bodyCallback, this );
    head_scalar_sub_ = nh_.subscribe<geometry_msgs::AccelStamped>( "head_scalar", 1, &Control::headCallback, this );
    state_sub_ = nh_.subscribe<std_msgs::Bool>( "state", 1, &Control::stateCallback, this );
    imu_override_sub_ = nh_.subscribe<std_msgs::Bool>( "imu_override", 1, &Control::imuOverrideCallback, this );
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>( "imu/data", 1, &Control::imuCallback, this );
    sounds_pub_ = nh_.advertise<hexapod_msgs::Sounds>( "sounds", 1 );
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>( "joint_states", 1 );
    // Ping the imu to re-calibrate
    imu_calibrate_ = nh_.serviceClient<std_srvs::Empty>("imu/calibrate");
    imu_calibrate_.call( calibrate_ );
}

//==============================================================================
// Getter and Setters
//==============================================================================

void Control::setHexActiveState( bool state )
{
    hex_state_ = state;
}

bool Control::getHexActiveState( void )
{
    return hex_state_;
}

void Control::setPrevHexActiveState( bool state )
{
    prev_hex_state_ = state;
}

bool Control::getPrevHexActiveState( void )
{
    return prev_hex_state_;
}

//==============================================================================
// Joint State Publisher --- Sadly not dynamic yet
//==============================================================================
void Control::publishJointStates( const hexapod_msgs::LegsJoints &legs, const hexapod_msgs::RPY &head, sensor_msgs::JointState *joint_state )
{
    joint_state->header.stamp = ros::Time::now();
    int i = 0;
    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        joint_state->name[i] = static_cast<std::string>( servo_names_[i] );
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].coxa;
        i++;
        joint_state->name[i] = static_cast<std::string>( servo_names_[i] );
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].femur;
        i++;
        joint_state->name[i] = static_cast<std::string>( servo_names_[i] );
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tibia;
        i++;
        joint_state->name[i] = static_cast<std::string>( servo_names_[i] );
        joint_state->position[i] = servo_orientation_[i] * legs.leg[leg_index].tarsus;
        i++;
    }

    for( int head_index = 0; head_index < NUMBER_OF_HEAD_JOINTS; head_index++ )
    {
        joint_state->name[i] = static_cast<std::string>( servo_names_[i] );
        joint_state->position[i] = head_.yaw;
        i++;
    }
    joint_state_pub_.publish( *joint_state );
}

//==============================================================================
// Topics we subscribe to
//==============================================================================
//==============================================================================
// Base link movement callback
//==============================================================================

void Control::cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg )
{
    cmd_vel_.linear.x = cmd_vel_msg->linear.x;
    cmd_vel_.linear.y = cmd_vel_msg->linear.y;
    cmd_vel_.angular.z = cmd_vel_msg->angular.z;
}

//==============================================================================
// Base link movement callback
//==============================================================================

void Control::baseCallback( const geometry_msgs::AccelStampedConstPtr &base_scalar_msg )
{
    ros::Time current_time = ros::Time::now();
    double time_delta = current_time.toSec() - base_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        base_.x = base_scalar_msg->accel.linear.x * ( CYCLE_MAX_TRAVEL / 2 );
        base_.y = base_scalar_msg->accel.linear.y * ( CYCLE_MAX_TRAVEL / 2 );
        base_.theta = base_scalar_msg->accel.angular.z * CYCLE_MAX_YAW;
    }
}

//==============================================================================
// Override IMU and manipulate body orientation callback
//==============================================================================

void Control::bodyCallback( const geometry_msgs::AccelStampedConstPtr &body_scalar_msg )
{
    ros::Time current_time = ros::Time::now();
    double time_delta = current_time.toSec() - body_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        if( imu_override_.data == true )
        {
            body_.orientation.roll = ( body_scalar_msg->accel.angular.x * BODY_MAX_ROLL )* 0.01 + ( body_.orientation.roll * ( 1.0 - 0.01 ) );
            body_.orientation.pitch  = ( body_scalar_msg->accel.angular.y * BODY_MAX_PITCH ) * 0.01 + ( body_.orientation.pitch * ( 1.0 - 0.01 ) );
        }
    }
}

//==============================================================================
// Pan head callback
//==============================================================================

void Control::headCallback( const geometry_msgs::AccelStampedConstPtr &head_scalar_msg )
{
    ros::Time current_time = ros::Time::now();
    double time_delta = current_time.toSec() - head_scalar_msg->header.stamp.toSec();
    if ( time_delta < 1.0 ) // Don't move if timestamp is stale over a second
    {
        head_.yaw = head_scalar_msg->accel.angular.z * HEAD_MAX_PAN;
    }
}

//==============================================================================
// Active state callback - currently simple on/off - stand/sit
//==============================================================================

void Control::stateCallback( const std_msgs::BoolConstPtr &state_msg )
{
    if(state_msg->data == true )
    {
        if( getHexActiveState() == false )
        {
            // Activating hexapod
            body_.position.y = 0.0;
            body_.position.z = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            base_.y = 0.0;
            base_.x = 0.0;
            base_.theta = 0.0;
            setHexActiveState( true );
            ROS_INFO("Hexapod locomotion is now active.");
            sounds_.stand = true;
            sounds_pub_.publish( sounds_ );
            sounds_.stand = false;
        }
    }

    if( state_msg->data == false )
    {
        if( getHexActiveState() == true )
        {
            // Sit down hexapod
            body_.position.y = 0.0;
            body_.position.x = 0.0;
            body_.orientation.pitch = 0.0;
            body_.orientation.yaw = 0.0;
            body_.orientation.roll = 0.0;
            base_.y = 0.0;
            base_.x = 0.0;
            base_.theta = 0.0;
            setHexActiveState( false );
            ROS_WARN("Hexapod locomotion shutting down servos.");
            sounds_.shut_down = true;
            sounds_pub_.publish( sounds_ );
            sounds_.shut_down = false;
        }
    }
}

//==============================================================================
// IMU override callback
//==============================================================================

void Control::imuOverrideCallback( const std_msgs::BoolConstPtr &imu_override_msg )
{
    imu_override_.data = imu_override_msg->data;
}

//==============================================================================
// IMU callback to autolevel body if on angled ground
//==============================================================================

void Control::imuCallback( const sensor_msgs::ImuConstPtr &imu_msg )
{
    if( imu_override_.data == false )
    {
        const geometry_msgs::Vector3 &lin_acc = imu_msg->linear_acceleration;

        if( imu_init_stored_ == false )
        {
            imu_roll_init_ = -atan2( lin_acc.x, sqrt( lin_acc.y * lin_acc.y + lin_acc.z * lin_acc.z ) ); // flipped due to orientation of sensor
            imu_pitch_init_ = -atan2( lin_acc.y, lin_acc.z );
            imu_pitch_init_ = ( imu_pitch_init_ >= 0.0 ) ? ( PI - imu_pitch_init_ ) : ( -imu_pitch_init_ - PI );
            imu_init_stored_ = true;
        }

        imu_roll_lowpass_ = lin_acc.x * 0.01 + ( imu_roll_lowpass_ * ( 1.0 - 0.01 ) );
        imu_pitch_lowpass_ = lin_acc.y * 0.01 + ( imu_pitch_lowpass_ * ( 1.0 - 0.01 ) );
        imu_yaw_lowpass_ = lin_acc.z * 0.01 + ( imu_yaw_lowpass_ * ( 1.0 - 0.01 ) );

        double imu_roll = -atan2( imu_roll_lowpass_, sqrt( imu_pitch_lowpass_ * imu_pitch_lowpass_ + imu_yaw_lowpass_ * imu_yaw_lowpass_ ) );
        double imu_pitch = -atan2( imu_pitch_lowpass_, imu_yaw_lowpass_ );
        imu_pitch = ( imu_pitch >= 0.0 ) ? ( PI - imu_pitch ) : ( -imu_pitch - PI );

        double imu_roll_delta = imu_roll_init_ - imu_roll;
        double imu_pitch_delta = imu_pitch_init_ - imu_pitch;

        if( ( std::abs( imu_roll_delta ) > 0.0872664626 ) || ( std::abs( imu_pitch_delta ) > 0.0872664626 ) )
        {
            sounds_.auto_level = true;
            sounds_pub_.publish( sounds_ );
            sounds_.auto_level = false;
        }

        if( imu_roll_delta < -0.0174532925 ) // 1 degree
        {
            if( body_.orientation.roll < 0.209 )  // 12 degrees limit
            {
                body_.orientation.roll = body_.orientation.roll + 0.0002; // 0.01 degree increments
            }
        }

        if( imu_roll_delta > 0.0174532925 ) // 1 degree
        {
            if( body_.orientation.roll > -0.209 )  // 12 degrees limit
            {
                body_.orientation.roll = body_.orientation.roll - 0.0002;  // 0.01 degree increments
            }
        }

        if( imu_pitch_delta < -0.0174532925 ) // 1 degree
        {
            if( body_.orientation.pitch < 0.209 )  // 12 degrees limit
            {
                body_.orientation.pitch = body_.orientation.pitch + 0.0002;  // 0.01 degree increments
            }
        }

        if( imu_pitch_delta > 0.0174532925 ) // 1 degree
        {
            if( body_.orientation.pitch > -0.209 ) // 12 degrees limit
            {
                body_.orientation.pitch = body_.orientation.pitch - 0.0002;  // 0.01 degree increments
            }
        }
    }
}
