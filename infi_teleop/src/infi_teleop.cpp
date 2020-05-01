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

#include <infi_teleop/infi_teleop.h>
#include <infi_teleop/joy_profile.h>


InfiTeleop::InfiTeleop()
{
    std::string driving_topic;
            //torso_elevator_sim_topic,
            //torso_elevator_real_topic;
            //head_topic,
            //gripper_topic;

    ros::param::get("~tele_arm", tele_arm_);

    if (!ros::param::get("~topics/driving", driving_topic))
    {
        ROS_ERROR("[infi_teleop]: topics params are missing. did you load topics.yaml?");
        exit(EXIT_FAILURE);
    }
    
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(driving_topic, 5);
    //torso_real_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_real_topic, 5);
    //torso_sim_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_sim_topic, 5);
    //head_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(head_topic, 5);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &InfiTeleop::joyCallback, this);
    //joints_states_sub_ = nh_.subscribe("joint_states", 5, &Armadillo2Teleop::jointsUpdateCB, this);
    //gripper_sub_ = nh_.subscribe("/gripper_controller/current_gap", 5, &Armadillo2Teleop::gripperGapCB, this);

    /// /gripper_controller/current_gap

    /* limits */
    ros::param::get("~limits/torso/lower", joy_.torso.limit_lower);
    ros::param::get("~limits/torso/upper", joy_.torso.limit_upper);

    /*
    ros::param::get("~limits/head/pan_lower", joy_.head.limit_lower_pan);
    ros::param::get("~limits/head/pan_upper", joy_.head.limit_upper_pan);
    ros::param::get("~limits/head/tilt_lower", joy_.head.limit_lower_tilt);
    ros::param::get("~limits/head/tilt_upper", joy_.head.limit_upper_tilt);
*/
    //ros::param::get("~limits/gripper/limit_lower", joy_.gripper.limit_lower);
    //ros::param::get("~limits/gripper/limit_upper", joy_.gripper.limit_upper);
    //ros::param::get("~limits/gripper/max_effort", joy_.gripper.goal.command.max_effort);

    /* load joystick profile */
    std::string profile_name = "logitech_F310";
    ros::param::get("~profile", profile_name);
    loadProfile(profile_name);

    ROS_INFO("[infi_teleop]: ready to dance according to joy profile: %s", profile_name.c_str());
}

/*
void InfiTeleop::gripperGapCB(const std_msgs::Float32::ConstPtr &msg)
{
    if (!tele_arm_) return;
    joy_.gripper.goal.command.position = msg->data;
    joy_.gripper.init_state_recorded = true;
    gripper_sub_.shutdown();
}*/

void InfiTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    update(joy);
}

void InfiTeleop::drive()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = joy_.twist.axis_val_angular;
    twist_msg.linear.x = joy_.twist.axis_val_linear;
    twist_pub_.publish(twist_msg);
}
/*
void InfiTeleop::moveTorso()
{
    std_msgs::Float64 torso_pos;
    torso_pos.data = joy_.torso.axis_val_updown;
    torso_real_pub_.publish(torso_pos);
    torso_sim_pub_.publish(torso_pos);
}*/
/*
void InfiTeleop::moveHead()
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=joy_.head.axis_val_pan * M_PI / 180;
    q_goal[1]=joy_.head.axis_val_tilt * M_PI / 180;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    head_pub_.publish(traj);
}*/

bool InfiTeleop::loadProfile(const std::string &profile_name)
{
    /* twist */
    ros::param::get("~" + profile_name + "/twist/joy_axis_linear", joy_.twist.joy_axis_linear);
    ros::param::get("~" + profile_name + "/twist/joy_axis_angular", joy_.twist.joy_axis_angular);
    ros::param::get("~" + profile_name + "/twist/scale_angular", joy_.twist.scale_angular);
    ros::param::get("~" + profile_name + "/twist/scale_linear", joy_.twist.scale_linear);
    /* torso */
    ros::param::get("~" + profile_name + "/torso/joy_axis_updown", joy_.torso.joy_axis_updown);
    ros::param::get("~" + profile_name + "/torso/increment", joy_.torso.increment);

    /* head */
    /*
    ros::param::get("~" + profile_name + "/head/right_btn", joy_.head.joy_btn_pan_right);
    ros::param::get("~" + profile_name + "/head/left_btn", joy_.head.joy_btn_pan_left);
    ros::param::get("~" + profile_name + "/head/up_btn", joy_.head.joy_btn_tilt_up);
    ros::param::get("~" + profile_name + "/head/down_btn", joy_.head.joy_btn_tilt_down);
    ros::param::get("~" + profile_name + "/head/inc_pan", joy_.head.inc_pan);
    ros::param::get("~" + profile_name + "/head/inc_tilt", joy_.head.inc_tilt);
	*/
    /* arm */
    /*
    ros::param::get("~" + profile_name + "/arm/rotation1_axis", joy_.arm.joy_axis_rotation1);
    ros::param::get("~" + profile_name + "/arm/shoulder1_axis", joy_.arm.joy_axis_shoulder1);
    ros::param::get("~" + profile_name + "/arm/shoulder2_axis", joy_.arm.joy_axis_shoulder2);
    ros::param::get("~" + profile_name + "/arm/rotation2_axis", joy_.arm.joy_axis_rotation2);
    ros::param::get("~" + profile_name + "/arm/shoulder3_up_btn", joy_.arm.joy_btn_shoulder3_up);
    ros::param::get("~" + profile_name + "/arm/shoulder3_down_btn", joy_.arm.joy_btn_shoulder3_down);
    ros::param::get("~" + profile_name + "/arm/wrist_cw_btn", joy_.arm.joy_btn_wrist_cw);
    ros::param::get("~" + profile_name + "/arm/wrist_ccw_btn", joy_.arm.joy_btn_wrist_ccw);
    ros::param::get("~" + profile_name + "/arm/reset_arm", joy_.arm.joy_btn_reset);
    ros::param::get("~" + profile_name + "/arm/inc", joy_.arm.increment);
	*/
    /* gripper */
    /*
    ros::param::get("~" + profile_name + "/gripper/axis", joy_.gripper.joy_axis);
    ros::param::get("~" + profile_name + "/gripper/inc", joy_.gripper.increment);
    */

    /* utils */
    //ros::param::get("~" + profile_name + "/arm_mode_btn", joy_.utils.joy_btn_arm_mode);
    ros::param::get("~" + profile_name + "/safety_btn", joy_.utils.joy_btn_safety);
}
/*
void InfiTeleop::moveGripper()
{
    if (!tele_arm_) return;
    gripper_client_->sendGoal(joy_.gripper.goal);
}
*/
/*
void InfiTeleop::moveArm()
{
}
*/
/*
void InfiTeleop::resetArm()
{
}
*/

void InfiTeleop::update(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
	//ROS_INFO("[infi_teleop]: axis forward: %f \ angular: %f", joy_msg->axes[joy_.twist.joy_axis_linear], joy_msg->axes[joy_.twist.joy_axis_angular]);
	/*
    if (!joy_msg->buttons[joy_.utils.joy_btn_safety])
        return;
	*/
	
    //if (!joy_msg->buttons[joy_.utils.joy_btn_arm_mode])
    {
        /* drive robot */
        joy_.twist.axis_val_angular = joy_msg->axes[joy_.twist.joy_axis_angular];
        joy_.twist.axis_val_angular *= joy_.twist.scale_angular;

        joy_.twist.axis_val_linear = joy_msg->axes[joy_.twist.joy_axis_linear];
        joy_.twist.axis_val_linear *= joy_.twist.scale_linear;
        drive();

    }
}
