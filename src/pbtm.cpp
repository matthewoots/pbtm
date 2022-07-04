/*
* pbtm.cpp
*
* ---------------------------------------------------------------------
* Created by Matthew (matthewoots@gmail.com) in 2022
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include <pbtm.h>
#include <helper.h>

using namespace helper;

/** @brief Get current uav pose */
void pbtm_class::pose_callback(
	const geometry_msgs::PoseStampedConstPtr& msg)
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex);

	_last_pose_time = ros::Time::now();
   	
	// local position in enu frame
	current_transform_enu.translation() = Vector3d(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z
	);
	// local rotation in enu frame
	current_transform_enu.linear() = Quaterniond(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z).toRotationMatrix();

	Eigen::Vector3d local_enu_euler = euler_rpy(current_transform_enu.linear());
	
	// global nwu affine matrix
	global_nwu_pose = local_to_global_t * current_transform_enu;
}

/** 
* @brief send_command via mavros to flight controller
* Update pbtm_class::state_command cmd_nwu first and including cmd_nwu.q for yaw command
*/
void pbtm_class::send_command()
{	
	std::lock_guard<std::mutex> send_command_lock(send_command_mutex);

	Eigen::Affine3d global_to_local_setpoint;

    global_to_local_setpoint.translation() = cmd_nwu.pos;
	global_to_local_setpoint.linear() = cmd_nwu.q.toRotationMatrix();

	Eigen::Affine3d enu_cmd_pose = 
        global_to_local_t * global_to_local_setpoint;

	Eigen::Vector3d enu_cmd_vel =
		enu_to_nwu().toRotationMatrix() * cmd_nwu.vel;
	Eigen::Vector3d enu_cmd_acc =
		enu_to_nwu().toRotationMatrix() * cmd_nwu.acc;

	mavros_msgs::PositionTarget _cmd;

	_cmd.header.stamp = ros::Time::now();
	_cmd.position = vector_to_point(enu_cmd_pose.translation());
	_cmd.velocity = vector_to_ros_vector(enu_cmd_vel);
	_cmd.acceleration_or_force = vector_to_ros_vector(enu_cmd_acc);

	Eigen::Vector3d nwu_cmd_euler = euler_rpy(enu_cmd_pose.linear());

	double cmd_yaw = nwu_cmd_euler.z();
	_cmd.yaw = (float)constrain_between_180(cmd_yaw);

	// For the type mask we have to ignore the rest (3520)
	// 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
	// 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
	// 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
	// 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
	// 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate
	_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	// pos_sp.type_mask = 3576; // Ignore Velocity, Acceleration and Yaw
	// pos_sp.type_mask = 2552; // Ignore Velocity, Acceleration
	_cmd.type_mask = 2496; // Ignore Acceleration
	// pos_sp.type_mask = 3520; // Ignore Acceleration and Yaw
	// pos_sp.type_mask = 3072; // Ignore Yaw
	// pos_sp.type_mask = 2048;
	_local_pos_raw_pub.publish(_cmd);

}

bool pbtm_class::check_sent_command(double tolerance)
{
	// check_sent_command is true if command is within the tolerance
	std::lock_guard<std::mutex> send_command_lock(send_command_mutex);
	ros::Duration duration = ros::Time::now() - _prev_command_time;
	return duration.toSec() < tolerance;
}

void pbtm_class::waypoint_command_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	trajectory_msgs::JointTrajectory copy_msg = *msg;
	std::string _agent_id = copy_msg.joint_names[0];
	if (_agent_id.compare(_id) != 0)
		return;
	
	int waypoint_command_type = joint_trajectory_to_waypoint(copy_msg);
	
	if (waypoint_command_type >= 0)
	{
		wp_pos_vector.clear();
		uav_task = waypoint_command_type;
	}
	else
		printf("%sdrone%d%s rejected invalid mission type! \n",
			KGRN, uav_id, KNRM);
}

void pbtm_class::set_offboard()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    _prev_command_time = ros::Time::now();

    // *** Make sure takeoff is not immediately sent, this will help to stream the correct data to the program first
    // *** Will give a 5sec buffer ***
    while (ros::Time::now() - last_request > ros::Duration(5.0))
    {
		// Empty
    }

	home_transformation = global_nwu_pose;
	Eigen::Vector3d current_nwu_pos = home_transformation.translation();
    current_nwu_pos.z() -= 0.05;

	Eigen::Quaterniond q(home_transformation.linear());

	cmd_nwu.pos = current_nwu_pos;
	cmd_nwu.vel = Eigen::Vector3d::Zero();
	cmd_nwu.acc = Eigen::Vector3d::Zero();
	cmd_nwu.q = q;

	// send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i)
    {
        send_command();
        ros::spinOnce();
        rate.sleep();
    }

	mavros_msgs::State uav_current_state;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_mode_ready = false;
    last_request = ros::Time::now();

    while (!is_mode_ready)
    {
        if (uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            printf("%s[pbtm.cpp] Try set offboard \n", KYEL);
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                printf("%s[pbtm.cpp] Offboard Enabled \n", KGRN);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!uav_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                printf("%s[pbtm.cpp] Try arm \n", KYEL);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("%s[pbtm.cpp] Vehicle armed \n", KGRN);  
                }
                last_request = ros::Time::now();
            }
        }

        send_command();

        is_mode_ready = (uav_current_state.mode == "OFFBOARD") && uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();        
    }

	_offboard_enabled = true;
    printf("%s[pbtm.cpp] Offboard mode activated! \n", KBLU);

    return;
}

void pbtm_class::drone_timer(const ros::TimerEvent &)
{

	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	// if uav has taken off, the manager cannot takeoff again
	if (uav_task == kTakeOff && _offboard_enabled)
		uav_task = kHover;

	switch (uav_task)
    {
		case kTakeOff:
		{
			if (!_offboard_enabled)
				set_offboard();
		}

		case kHover:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			break;
		}

		case kMission:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			break;
		}

		case kHome:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			break;
		}

		case kLand:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				break;
			}

			break;
		}

		default:
			break;
		

	}
}

void pbtm_class::stop_and_hover()
{	
	std::lock_guard<std::mutex> send_command_lock(send_command_mutex);

	// By default cmd pos message would not change;
	// cmd_nwu.pos = cmd_nwu.pos;
	cmd_nwu.vel = Eigen::Vector3d::Zero();
	cmd_nwu.acc = Eigen::Vector3d::Zero();
}

/** 
* @brief joint_trajectory_to_waypoint command and waypoints message translation
* @param points.positions = position waypoint
* @param points.time_from_start = command type 
*/
int pbtm_class::joint_trajectory_to_waypoint(trajectory_msgs::JointTrajectory jt)
{
	// Size of joint trajectory
	int size_of_vector = (int)jt.points.size();
	int mission_type = (int)(jt.points[0].time_from_start.toSec());

	// VehicleTask not within kIdle to kLand
	if (mission_type > 5 || mission_type < 0)
	{
		printf("%sdrone%d%s invalid mission_type : %s%d%s! \n", 
			KGRN, uav_id, KNRM, 
			KRED, mission_type, KRED);
		return -1;
	}

	for (int i = 0; i < size_of_vector; i++)
	{
		pbtm_class::state_command s;
		if (!jt.points[i].positions.empty())
		{
			double height_setpoint = 
				max(min(jt.points[i].positions[2], height_list[1]), height_list[0]);
			s.pos = Eigen::Vector3d(jt.points[i].positions[0],
					jt.points[i].positions[1], height_setpoint);
		}
		wp_pos_vector.push_back(s.pos);
	}

	return mission_type;
}