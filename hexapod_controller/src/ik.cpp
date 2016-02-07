
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


#include <ik.h>

static const double PI = atan(1.0)*4.0;

//==============================================================================
//  Constructor: Initialize ik variables
//==============================================================================

Ik::Ik( void )
{
    //=============================================================================
    // Define Physical Measurements in mm <config file>
    //=============================================================================

    ros::param::get( "COXA_TO_CENTER_X", COXA_TO_CENTER_X );
    ros::param::get( "COXA_TO_CENTER_Y", COXA_TO_CENTER_Y );
    ros::param::get( "INIT_COXA_ANGLE", INIT_COXA_ANGLE );
    ros::param::get( "INIT_FOOT_POS_X", INIT_FOOT_POS_X );
    ros::param::get( "INIT_FOOT_POS_Y", INIT_FOOT_POS_Y );
    ros::param::get( "INIT_FOOT_POS_Z", INIT_FOOT_POS_Z );
    ros::param::get( "COXA_LENGTH", COXA_LENGTH );
    ros::param::get( "FEMUR_LENGTH", FEMUR_LENGTH );
    ros::param::get( "TIBIA_LENGTH", TIBIA_LENGTH );
    ros::param::get( "TARSUS_LENGTH", TARSUS_LENGTH );
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
}

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle
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

void Ik::calculateIK( const hexapod_msgs::FeetPositions &feet, const hexapod_msgs::Pose &body, hexapod_msgs::LegsJoints *legs )
{
    double sign = -1.0;
    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        if( leg_index <= 2 )
        {
            sign = -1.0;
        }
        else
        {
            sign = 1.0;
        }

        // First calculate sinus and co-sinus for each angular axis
        Trig A = getSinCos( body.orientation.yaw + feet.foot[leg_index].orientation.yaw );
        Trig B = getSinCos( body.orientation.pitch );
        Trig G = getSinCos( body.orientation.roll );

        // Calculating totals from the feet to center of the body
        double cpr_x = feet.foot[leg_index].position.x + body.position.x - INIT_FOOT_POS_X[leg_index] - COXA_TO_CENTER_X[leg_index];

        double cpr_y = feet.foot[leg_index].position.y + sign*( body.position.y + INIT_FOOT_POS_Y[leg_index] + COXA_TO_CENTER_Y[leg_index] );

        double cpr_z = feet.foot[leg_index].position.z + body.position.z + TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index];


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
        double feet_pos_x = -INIT_FOOT_POS_X[leg_index] + body.position.x - body_pos_x + feet.foot[leg_index].position.x;
        double feet_pos_y =  INIT_FOOT_POS_Y[leg_index] + sign*( body.position.y - body_pos_y + feet.foot[leg_index].position.y );
        double feet_pos_z =  INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH + body.position.z - body_pos_z - feet.foot[leg_index].position.z;

        // Length between the Root and Foot Position ...Pythagorean theorem
        double femur_to_tarsus = sqrt( pow( feet_pos_x, 2 ) + pow( feet_pos_y, 2 ) ) - COXA_LENGTH;

        if( std::abs( femur_to_tarsus ) > ( FEMUR_LENGTH + TIBIA_LENGTH ) )
        {
            ROS_FATAL("IK Solver cannot solve a foot position that is not within leg reach!!!");
            ROS_FATAL("Shutting down so configuration can be fixed!!!");
            ros::shutdown();
            break;
        }

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

