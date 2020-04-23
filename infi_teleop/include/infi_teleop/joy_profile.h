/*******************************************************************************
* Copyright (c) 2020, infibotics LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Shmulik Edelman*/

#ifndef INFI_TELEOP_JOY_PROFILE_H
#define INFI_TELEOP_JOY_PROFILE_H

struct joy_puppet
{
    bool init_state_recorded = false;
};

struct joy_twist : joy_puppet
{
    float axis_val_linear = 0;
    float axis_val_angular = 0;

    float scale_angular = 0;
    float scale_linear = 0;

    int joy_axis_linear = 0;
    int joy_axis_angular = 0;
};

struct joy_torso : joy_puppet
{
    float axis_val_updown = 0;
    float increment = 0; //meters

    int joy_axis_updown = 0;

    float limit_upper = 0;
    float limit_lower = 0;
};

struct joy_utils
{
    int joy_btn_arm_mode = 0;
    int joy_btn_safety = 0;
};

struct joy_profile
{
    joy_torso torso;
    joy_twist twist;
    joy_utils utils;
};


#endif //Infi_TELEOP_JOY_H
