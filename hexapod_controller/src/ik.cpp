
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

#include <ik.h>

static const double PI = atan(1.0)*4.0;

//==============================================================================
//  Constructor: Initialize ik variables
//==============================================================================

Ik::Ik( void )
{
    //=============================================================================
    // Distance in mm from center of the body to the coxa joint <launch file>
    //=============================================================================

    ros::param::get( "RR_OFFSET_X", BODY_CENTER_TO_COXA_X[0] );
    ros::param::get( "RM_OFFSET_X", BODY_CENTER_TO_COXA_X[1] );
    ros::param::get( "RF_OFFSET_X", BODY_CENTER_TO_COXA_X[2] );
    ros::param::get( "LR_OFFSET_X", BODY_CENTER_TO_COXA_X[3] );
    ros::param::get( "LM_OFFSET_X", BODY_CENTER_TO_COXA_X[4] );
    ros::param::get( "LF_OFFSET_X", BODY_CENTER_TO_COXA_X[5] );

    ros::param::get( "RR_OFFSET_Y", BODY_CENTER_TO_COXA_Y[0] );
    ros::param::get( "RM_OFFSET_Y", BODY_CENTER_TO_COXA_Y[1] );
    ros::param::get( "RF_OFFSET_Y", BODY_CENTER_TO_COXA_Y[2] );
    ros::param::get( "LR_OFFSET_Y", BODY_CENTER_TO_COXA_Y[3] );
    ros::param::get( "LM_OFFSET_Y", BODY_CENTER_TO_COXA_Y[4] );
    ros::param::get( "LF_OFFSET_Y", BODY_CENTER_TO_COXA_Y[5] );

    //=============================================================================
    // Define Initial Coxa Offsets in Degrees <launch file>
    //=============================================================================

    ros::param::get( "RR_COXA_ANGLE", INIT_COXA_ANGLE[0] );
    ros::param::get( "RM_COXA_ANGLE", INIT_COXA_ANGLE[1] );
    ros::param::get( "RF_COXA_ANGLE", INIT_COXA_ANGLE[2] );
    ros::param::get( "LR_COXA_ANGLE", INIT_COXA_ANGLE[3] );
    ros::param::get( "LM_COXA_ANGLE", INIT_COXA_ANGLE[4] );
    ros::param::get( "LF_COXA_ANGLE", INIT_COXA_ANGLE[5] );

    //=============================================================================
    // Define distance in mm from center of the body to the coxa joint <launch file>
    //=============================================================================

    ros::param::get( "RR_INIT_FOOT_POS_X", INIT_FOOT_POS_X[0] );
    ros::param::get( "RM_INIT_FOOT_POS_X", INIT_FOOT_POS_X[1] );
    ros::param::get( "RF_INIT_FOOT_POS_X", INIT_FOOT_POS_X[2] );
    ros::param::get( "LR_INIT_FOOT_POS_X", INIT_FOOT_POS_X[3] );
    ros::param::get( "LM_INIT_FOOT_POS_X", INIT_FOOT_POS_X[4] );
    ros::param::get( "LF_INIT_FOOT_POS_X", INIT_FOOT_POS_X[5] );

    ros::param::get( "RR_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[0] );
    ros::param::get( "RM_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[1] );
    ros::param::get( "RF_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[2] );
    ros::param::get( "LR_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[3] );
    ros::param::get( "LM_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[4] );
    ros::param::get( "LF_INIT_FOOT_POS_Y", INIT_FOOT_POS_Y[5] );

    ros::param::get( "FOOT_INIT_Z", INIT_FOOT_POS_Z );

    //=============================================================================
    // Define Leg Measurements in mm <launch file>
    //=============================================================================

    ros::param::get( "COXA_LENGTH", COXA_LENGTH );
    ros::param::get( "FEMUR_LENGTH", FEMUR_LENGTH );
    ros::param::get( "TIBIA_LENGTH", TIBIA_LENGTH );
    ros::param::get( "TARSUS_LENGTH", TARSUS_LENGTH );
}

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle +/- multiple circles
//=============================================================================

Trig Ik::getSinCos( double angle_rad )
{
    Trig body_trig;

    body_trig.sine = sin( angle_rad );
    body_trig.cosine = cos( angle_rad );

    return body_trig;
}

//=============================================================================
// Inverse Kinematics
//=============================================================================

void Ik::calculateIK( const hexapod_msgs::FeetPositions &feet, const hexapod_msgs::BodyJoint &body, hexapod_msgs::LegsJoints *legs )
{
    char sign = -1;
    for( int leg_index = 0; leg_index <= 5; leg_index++ )
    {
        if( leg_index <= 2 )
        {
            sign = -1;
        }
        else
        {
            sign = 1;
        }

        // First calculate sinus and co-sinus for each angular axis
        Trig A = getSinCos( body.yaw + feet.foot[leg_index].yaw );
        Trig B = getSinCos( body.pitch );
        Trig G = getSinCos( body.roll );

        // Calculating totals from center of the body to the feet
        double cpr_x = BODY_CENTER_TO_COXA_X[leg_index] + INIT_FOOT_POS_X[leg_index] + body.x + feet.foot[leg_index].x;
        double cpr_y = BODY_CENTER_TO_COXA_Y[leg_index] + sign*( INIT_FOOT_POS_Y[leg_index] + body.y ) + feet.foot[leg_index].y;
        double cpr_z = INIT_FOOT_POS_Z - TARSUS_LENGTH + body.z - feet.foot[leg_index].z;

        // Calculation of angular matrix of body (Tait-Bryan angles Z, Y, X)
        // http://en.wikipedia.org/wiki/Euler_angles

        double body_pos_x = cpr_x - ( ( cpr_x * A.cosine * B.cosine ) +
                                      ( cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine ) +
                                      ( cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine  )
                                    );

        double body_pos_y = cpr_y - ( ( cpr_x * B.cosine * A.sine ) +
                                      ( cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine ) +
                                      ( cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine  )
                                    );

        double body_pos_z = cpr_z - ( ( -cpr_x * B.sine ) + ( cpr_y * B.cosine * G.sine ) + ( cpr_z * B.cosine * G.cosine ) );

        // Calculate foot position
        double feet_pos_x = INIT_FOOT_POS_X[leg_index] + body.x - body_pos_x + feet.foot[leg_index].x;
        double feet_pos_y = INIT_FOOT_POS_Y[leg_index] + sign*( body.y - body_pos_y + feet.foot[leg_index].y );
        double feet_pos_z = INIT_FOOT_POS_Z - TARSUS_LENGTH + body.z - body_pos_z - feet.foot[leg_index].z;

        // Length between the Root and Foot Position ...Pythagorean theorem
        double femur_to_tarsus = sqrt( pow( feet_pos_x, 2 ) + pow( feet_pos_y, 2 ) ) - COXA_LENGTH;

        // Length of the sides of the triangle formed by the femur, tibia and tarsus joints.
        double side_a = FEMUR_LENGTH;
        double side_a_sqr = pow( FEMUR_LENGTH, 2 );

        double side_b = TIBIA_LENGTH;
        double side_b_sqr = pow( TIBIA_LENGTH, 2 );

        double side_c = sqrt( pow( femur_to_tarsus, 2 ) + pow( feet_pos_z, 2 ) );
        double side_c_sqr = pow( side_c, 2 );

        // We are using the law of cosines on the triangle formed by the femur, tibia and tarsus joints.
        double angle_b = acos( ( side_a_sqr - side_b_sqr + side_c_sqr ) / ( 2.0 * side_a * side_c ) );
        double angle_c = acos( ( side_a_sqr + side_b_sqr - side_c_sqr ) / ( 2.0 * side_a * side_b ) );

        // Angle of line between the femur and Tarsus joints with respect to feet_pos_z.
        double theta = atan2( femur_to_tarsus, feet_pos_z );

        // Resulting joint angles in radians.
        legs->leg[leg_index].coxa = atan2( feet_pos_x, feet_pos_y ) + INIT_COXA_ANGLE[leg_index];
        legs->leg[leg_index].femur = ( PI/2 ) - ( theta + angle_b );
        legs->leg[leg_index].tibia = ( PI/2 ) - angle_c;
        legs->leg[leg_index].tarsus = legs->leg[leg_index].femur + legs->leg[leg_index].tibia;
    }
}

