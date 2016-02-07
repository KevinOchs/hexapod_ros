
// ROS Hexapod Sound Node
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


#include <hexapod_sound.h>
#include <ros/package.h>

//==============================================================================
// Constructor
//==============================================================================

HexapodSound::HexapodSound( void )
{
    sound_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1, 0);
    sounds_sub_ = nh_.subscribe<hexapod_msgs::Sounds>( "/sounds", 1, &HexapodSound::soundsCallback, this);
    sound_package_path_ = ros::package::getPath("hexapod_sound");
}

void HexapodSound::soundsCallback( const hexapod_msgs::SoundsConstPtr &sounds_msg )
{
    if( sounds_msg->stand == true )
    {
        if( sounds_.stand != true )
        {
            sounds_.stand = true;
        }
    }

    if( sounds_msg->shut_down == true )
    {
        if( sounds_.shut_down != true )
        {
            sounds_.shut_down = true;
        }
    }

    if( sounds_msg->waiting == true )
    {
        if( sounds_.waiting != true )
        {
            sounds_.waiting = true;
        }
    }

    if( sounds_msg->auto_level == true )
    {
        if( sounds_.auto_level != true )
        {
            sounds_.auto_level = true;
        }
    }
}


void HexapodSound::playSoundFile(std::string sound_file, int delay_time)
{
    sound_req_.sound = sound_play::SoundRequest::PLAY_FILE;
    sound_req_.command = sound_play::SoundRequest::PLAY_ONCE;
    sound_req_.arg = sound_package_path_ + "/sounds/" + sound_file; // need to due this due to bug in sound_play
    sound_pub_.publish( sound_req_ );
    ros::Duration( delay_time ).sleep();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hexapod_sound");
    HexapodSound hexapodSound;

    hexapodSound.playSoundFile( "empty.ogg", 3 );
    hexapodSound.playSoundFile( "intelChime.ogg", 3 );
    hexapodSound.playSoundFile( "activeAwaitingCommands.ogg", 3 );

    ros::AsyncSpinner spinner( 1 ); // Using 1 threads
    spinner.start();
    ros::Rate loop_rate( 10 ); // 10 hz
    while( ros::ok() )
    {
        if( hexapodSound.sounds_.stand == true )
        {
            hexapodSound.playSoundFile( "standingUp.ogg", 3 );
            hexapodSound.sounds_.stand = false;
        }

        if( hexapodSound.sounds_.auto_level == true )
        {
            hexapodSound.playSoundFile( "autoLevelingBody.ogg", 6 );
            hexapodSound.sounds_.auto_level = false;
        }

        if( hexapodSound.sounds_.waiting == true )
        {
            hexapodSound.playSoundFile( "activeAwaitingCommands.ogg", 3 );
            hexapodSound.sounds_.waiting = false;
        }

        if( hexapodSound.sounds_.shut_down == true )
        {
            hexapodSound.playSoundFile( "shuttingDown.ogg", 3);
            hexapodSound.sounds_.shut_down = false;
        }

        loop_rate.sleep();
    }
}
