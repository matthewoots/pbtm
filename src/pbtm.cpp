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

/** @brief Get bypass message calculated from external path planning module */
void pbtm_class::bypass_callback(
	const mavros_msgs::PositionTarget::ConstPtr &msg)
{
	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	_prev_bypass_msg_time = ros::Time::now();

	if (uav_task != kBypass)
		return;
	
	mavros_msgs::PositionTarget bypass_msg = *msg;
	time_point<std::chrono::system_clock> now_time = 
		system_clock::now();
	double rel_now_time = duration<double>(now_time - stime).count();
	
	cmd_nwu.pos = Eigen::Vector3d(
		bypass_msg.position.x, bypass_msg.position.y, bypass_msg.position.z);
	cmd_nwu.t = rel_now_time;
	cmd_nwu.vel = Eigen::Vector3d(
		bypass_msg.velocity.x, bypass_msg.velocity.y, bypass_msg.velocity.z);
	double _norm = sqrt(pow(cmd_nwu.vel.x(),2) + pow(cmd_nwu.vel.y(),2));
	double _norm_x = cmd_nwu.vel.x() / _norm;
	double _norm_y = cmd_nwu.vel.y() / _norm;

	cmd_nwu.acc = Eigen::Vector3d(
		bypass_msg.acceleration_or_force.x, 
		bypass_msg.acceleration_or_force.y, 
		bypass_msg.acceleration_or_force.z);
	
	// If velocity is too low, then any noise will cause the yaw to fluctuate
	// Restrict the yaw if velocity is too low
	if (cmd_nwu.vel.norm() >= 0.15)
		last_yaw = atan2(_norm_y,_norm_x);
	// last_yaw = bypass_msg.yaw;

	cmd_nwu.q = 
		calculate_quadcopter_orientation(cmd_nwu.acc, last_yaw);
}


/** @brief Get current uav FCU state */
void pbtm_class::uavStateCallBack(
	const mavros_msgs::State::ConstPtr &msg)
{
	uav_current_state = *msg;
	// Make Sure FCU is connected, wait for 5s if not connected.
    // printf("%s[main.cpp] FCU Connection is %s %s\n", 
	// 	uav_current_state.connected? KBLU : KRED, 
	// 	uav_current_state.connected? "up" : "down", KNRM);
	if (!uav_current_state.connected)
	{
		printf("%s[main.cpp] Check for FCU connection %s\n", KRED, KNRM);
		return;
    }

	if (!uav_current_state.armed)
	{
		// printf("%s[main.cpp] uav disarmed %s\n", KRED, KNRM);
		_offboard_enabled = _setup = false;
    }

}

/** @brief Get current uav pose */
void pbtm_class::pose_callback(
	const geometry_msgs::PoseStampedConstPtr& msg)
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex);

	_last_pose_time = ros::Time::now();
	uav_local_pos_enu = point_to_vector(msg->pose.position);
   	
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
	
	uav_attitude_q = Vector4d(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z
	);

	Eigen::Vector3d local_enu_euler = euler_rpy(current_transform_enu.linear());
	
	// global nwu affine matrix
	global_nwu_pose = local_to_global_t * current_transform_enu;

	Eigen::Quaterniond q_nwu(global_nwu_pose.linear());
	geometry_msgs::PoseStamped global_nwu;
	global_nwu.header.stamp = msg->header.stamp;
	global_nwu.header.frame_id = "world";
	global_nwu.pose.position = vector_to_point(global_nwu_pose.translation());
	global_nwu.pose.orientation = quaternion_to_orientation(q_nwu);
	_pose_nwu_pub.publish(global_nwu);
}

void pbtm_class::vel_callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
	uav_local_vel_enu = ros_vector_to_vector(msg->twist.linear);
	uav_att_rate = ros_vector_to_vector(msg->twist.angular);
}

void pbtm_class::battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
	battery_volt = msg->voltage;
}

/** 
* @brief send_command via mavros to flight controller
* Update pbtm_class::state_command cmd_nwu first and including cmd_nwu.q for yaw command
*/
void pbtm_class::send_command()
{	
	Eigen::Affine3d global_to_local_setpoint;

    global_to_local_setpoint.translation() = cmd_nwu.pos;
	global_to_local_setpoint.linear() = cmd_nwu.q.toRotationMatrix();

	Eigen::Affine3d enu_cmd_pose = 
        global_to_local_t * global_to_local_setpoint;

	Eigen::Vector3d enu_cmd_vel =
		enu_to_nwu().toRotationMatrix() * cmd_nwu.vel;
	Eigen::Vector3d enu_cmd_acc =
		enu_to_nwu().toRotationMatrix() * cmd_nwu.acc;

	mavros_msgs::PositionTarget _cmd_enu;
	_cmd_enu.header.stamp = ros::Time::now();
	_cmd_enu.position = vector_to_point(enu_cmd_pose.translation());
	_cmd_enu.velocity = vector_to_ros_vector(enu_cmd_vel);
	_cmd_enu.acceleration_or_force = vector_to_ros_vector(enu_cmd_acc);

	_ref_enu_pub.publish(_cmd_enu);

	if (px4Ctrl)
	{
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
	// _cmd.type_mask = 2496; // Ignore Acceleration
	// pos_sp.type_mask = 3520; // Ignore Acceleration and Yaw
	// pos_sp.type_mask = 3072; // Ignore Yaw
    // send trajectory p,v,a to px4 (v and a are used as feed-forward)
    // https://docs.px4.io/main/en/flight_stack/controller_diagrams.html#combined-position-and-velocity-controller-diagram
    // https://docs.px4.io/main/en/flight_modes/offboard.html#copter-vtol
	_cmd.type_mask = 2048; // use p,v,a and ignore yaw_rate
	// _cmd.type_mask = 2496;
	
	_local_pos_raw_pub.publish(_cmd);
	}

	else
	{
		Eigen::Vector3d a_ref = enu_cmd_acc;

		Eigen::Vector3d enu_cmd_euler = euler_rpy(enu_cmd_pose.linear());

		double cmd_yaw = enu_cmd_euler.z();
		yaw_ref = (float)constrain_between_180(cmd_yaw);
		// reference attitude in quaternion
		Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, yaw_ref);
		Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

		Eigen::Vector3d pos_error = current_transform_enu.translation() - enu_cmd_pose.translation();
		Eigen::Vector3d vel_error = uav_local_vel_enu - enu_cmd_vel;

		Eigen::Vector3d a_des = pos_ctrl->calDesiredAcceleration(pos_error, vel_error, a_ref);

		if (_send_acceleration_setpoint)
		{
			// send acceleration command
			// works in sitl but not on physical hardware
			mavros_msgs::PositionTarget pos_sp;
			pos_sp.position.x = 0.0;
			pos_sp.position.y = 0.0;
			pos_sp.position.z = 0.0;
			pos_sp.velocity.x = 0.0;
			pos_sp.velocity.y = 0.0;
			pos_sp.velocity.z = 0.0;
			pos_sp.acceleration_or_force.x = a_des.x();
			pos_sp.acceleration_or_force.y = a_des.y();
			pos_sp.acceleration_or_force.z = a_des.z() - 9.81;
			pos_sp.yaw = yaw_ref; // fixed yaw
			pos_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
			pos_sp.type_mask = 2111; // 1+2+4+8+16+32+2048 i.e. only use acceleration setpoint
			_local_pos_raw_pub.publish(pos_sp);
			return;
		}

		q_des = pos_ctrl->calDesiredAttitude(a_des, yaw_ref);
		cmdBodyRate_(3) = pos_ctrl->calDesiredThrottle(a_des, uav_attitude_q, battery_volt, voltage_compensation_);

		mavros_msgs::AttitudeTarget msg;
		msg.header.stamp = ros::Time::now();
  		msg.header.frame_id = "map";
  		msg.body_rate.x = cmdBodyRate_(0);
  		msg.body_rate.y = cmdBodyRate_(1);
  		msg.body_rate.z = cmdBodyRate_(2);
  		msg.type_mask = 7;  // Ignore orientation messages (128); Ignore body rate messages (7)
  		msg.orientation.w = q_des(0);
  		msg.orientation.x = q_des(1);
  		msg.orientation.y = q_des(2);
  		msg.orientation.z = q_des(3);
  		msg.thrust = cmdBodyRate_(3);
		_att_rate_pub.publish(msg);
	}



}

bool pbtm_class::check_last_time(double tolerance, ros::Time time)
{
	// check_last_time is true if time is within the tolerance
	ros::Duration duration = ros::Time::now() - time;
	return duration.toSec() < tolerance;
}

void pbtm_class::waypoint_command_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	trajectory_msgs::JointTrajectory copy_msg = *msg;
	std::string _agent_id = copy_msg.joint_names[0];
	// printf("%s\n", _agent_id.c_str());

	if (_agent_id.compare(_id) != 0)
		return;
	
	if (!check_last_time(_timeout, _last_pose_time))
	{
		printf("[%sdrone%d%s pbtm.cpp] %sError in rate of local_position msg%s \n", 
			KGRN, uav_id, KNRM, KRED, KNRM);
		return;
	}
	
	int waypoint_command_type = joint_trajectory_to_waypoint(copy_msg);
	
	if (waypoint_command_type >= 0)
	{
		uav_task = waypoint_command_type;
		_setup = false;
	}
	else
		printf("%sdrone%d%s rejected invalid mission type! \n",
			KGRN, uav_id, KNRM);
}

void pbtm_class::set_offboard()
{
	arm_cmd.request.value = true;

    ros::Rate rate(_send_command_rate);
    ros::Time last_request = ros::Time::now();

    // *** Make sure takeoff is not immediately sent, this will help to stream the correct data to the program first
    // *** Will give a 1sec buffer ***
    while (ros::Time::now() - last_request < ros::Duration(1.0))
    {
		// Empty
    }

	home_transformation = global_nwu_pose;

	last_yaw = euler_rpy(home_transformation.linear()).z();
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

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_mode_ready = false;
    last_request = ros::Time::now();

    while (!is_mode_ready)
    {
		// printf("[%sdrone%d%s pbtm.cpp] %s! \n",
		// 	KGRN, uav_id, KNRM, uav_current_state.mode.c_str());

        if (uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            printf("[%sdrone%d%s pbtm.cpp] %sTry set offboard%s \n", 
				KGRN, uav_id, KNRM, KYEL, KNRM);
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                printf("[%sdrone%d%s pbtm.cpp] %sOffboard Enabled%s \n", 
					KGRN, uav_id, KNRM, KGRN, KNRM);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!uav_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                printf("[%sdrone%d%s pbtm.cpp] %sTry arm%s \n", 
					KGRN, uav_id, KNRM, KYEL, KNRM);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("[%sdrone%d%s pbtm.cpp] %sVehicle armed%s \n", 
					KGRN, uav_id, KNRM, KGRN, KNRM);  
                }
                last_request = ros::Time::now();
            }
        }

        send_command();

        is_mode_ready = (uav_current_state.mode == "OFFBOARD") 
			&& uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();        
    }

	_offboard_enabled = true;
    printf("[%sdrone%d%s pbtm.cpp] %sOffboard mode activated!%s \n", 
		KGRN, uav_id, KNRM, KBLU, KNRM);  

    return;
}

void pbtm_class::drone_timer(const ros::TimerEvent &)
{

	std::lock_guard<std::mutex> waypoint_lock(waypoint_command_mutex);

	// if uav has taken off, the manager cannot takeoff again
	// if (uav_task == kTakeOff && _offboard_enabled)
	// 	uav_task = kHover;

	switch (uav_task)
    {
		case kTakeOff: case kLand:
		{
			if (!_setup)
			{	
				path.poses.clear();

				if (!_offboard_enabled)
					set_offboard();

				initialize_bspline_server(takeoff_land_velocity);

				stime = std::chrono::system_clock::now();
				
				timespan.clear();
				timespan.push_back(0.0);
				timespan.push_back(_duration);
				
				_setup = true;

				printf("[%sdrone%d%s pbtm.cpp] kTakeoff/kLand %sFinished setting up bspline!%s \n", 
					KGRN, uav_id, KNRM, KBLU, KNRM); 
				visualize_log_path();
			}

			if (update_get_command_by_time())
			{
				if(!_set_takeoff_land_orientation)
				{
					_set_takeoff_land_orientation = true;
					_takeoff_land_orientation = global_nwu_pose.linear();
				}

				// fix yaw as before takeoff or land
				cmd_nwu.q = _takeoff_land_orientation;
				
				send_command();
			}

			else
			{
				printf("[%sdrone%d%s pbtm.cpp] kHover/kIdle \n", 
					KGRN, uav_id, KNRM);

				if (uav_task == kTakeOff)
					uav_task = kHover;
				else if (uav_task == kLand)
				{
					uav_task = kIdle;
					_offboard_enabled = false;
				}

				stop_and_hover();
				_setup = false;
				_set_takeoff_land_orientation = false;
			}

			break;
		}

		case kHover:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				uav_task = kIdle;
				break;
			}

			send_command();

			break;
		}

		case kBypass:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				uav_task = kIdle;
				break;
			}

			if (!_setup)
			{
				path.poses.clear();

				stime = std::chrono::system_clock::now();
				_setup = true;

				printf("[%sdrone%d%s pbtm.cpp] kBypass %sSetup Finished!%s \n", 
					KGRN, uav_id, KNRM, KBLU, KNRM); 
				
				bypass_timeout_start_time = ros::Time::now();
			}

			// Check for messages, if too long we start a timer before moving back to hover mode

			if (check_last_time(_bypass_timeout, _prev_bypass_msg_time))
			{
				bypass_timeout_start_time = ros::Time::now();
				send_command();
			}
			else
			{
				// Send hover command / last command
				send_command();
			}

			if (ros::Time::now() - bypass_timeout_start_time > ros::Duration(_bypass_timeout))
			{
				printf("[%sdrone%d%s pbtm.cpp] Timeout for bypass \n", 
					KGRN, uav_id, KNRM); 
				printf("[%sdrone%d%s pbtm.cpp] kHover \n", 
					KGRN, uav_id, KNRM); 
				uav_task = kHover;
				stop_and_hover();
				_setup = false;
			}

			break;
		}

		case kMission: case kHome:
		{
			if (!_offboard_enabled)
			{
				printf("%s[pbtm.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
				uav_task = kIdle;
				break;
			}

			if (!_setup)
			{
				path.poses.clear();
				initialize_bspline_server(_max_velocity);

				stime = std::chrono::system_clock::now();
				
				timespan.clear();
				timespan.push_back(0.0);
				timespan.push_back(_duration);
				
				_setup = true;

				printf("[%sdrone%d%s pbtm.cpp] kMission/kHover %sFinished setting up bspline!%s \n", 
					KGRN, uav_id, KNRM, KBLU, KNRM); 
				visualize_log_path();
			}

			if (update_get_command_by_time())
				send_command();
			else
			{
				printf("[%sdrone%d%s pbtm.cpp] kHover \n", 
					KGRN, uav_id, KNRM); 
				uav_task = kHover;
				stop_and_hover();
				_setup = false;
			}

			break;
		}

		case kIdle: default:
			break;
		

	}
}

void pbtm_class::initialize_bspline_server(double desired_velocity)
{
	if (wp_pos_vector.empty())
		return;
	
	double total_distance = 0.0;
	for (int i = 0; i < wp_pos_vector.size(); i++)
		total_distance += wp_pos_vector[i].norm();
	
	double est_duration = total_distance / desired_velocity;

	// Re-adjust duration so that our knots and divisions are matching
	double corrected_duration_secs = bsu.get_corrected_duration(
		_send_command_interval, est_duration);

	_knot_size = bsu.get_knots_size(
            _send_command_interval, corrected_duration_secs, _knot_division);

	_knot_interval = corrected_duration_secs / _knot_size;

	control_points.clear();
	control_points = ctt.uniform_distribution_of_cp(
        global_nwu_pose.translation(), wp_pos_vector, 
		_max_velocity, _knot_interval);
	
	// clamp start and also the end to stop uav at the beginning and the end
	for (int i = 0; i < _order; i++)
	{
		control_points.insert(control_points.begin(), 1, control_points[0]);
		control_points.push_back(control_points[control_points.size()-1]);
	}

	_duration = ((double)control_points.size() + (double)_order) * _knot_interval;
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
	if (mission_type > 6 || mission_type < 0)
	{
		printf("%sdrone%d%s invalid mission_type : %s%d%s! \n", 
			KGRN, uav_id, KNRM, 
			KRED, mission_type, KRED);
		return -1;
	}

	wp_pos_vector.clear();

	// VehicleTask enum list
	// kIdle 0
	// kTakeOff 1
	// kHover 2
	// kMission 3
	// kBypass 4
	// kHome 5
	// kLand 6
	if (mission_type == 3) // Mission
	{
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
	}
	else if (mission_type == 1) // Takeoff
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			global_nwu_pose.translation().x(),
			global_nwu_pose.translation().y(), 
			_takeoff_height));
	}
	if (mission_type == 4) // Bypass
	{
		// Publish to external path planning module
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.header.stamp = ros::Time::now();
		pose_msg.pose.position.x = jt.points[0].positions[0];
		pose_msg.pose.position.y = jt.points[0].positions[1];
		pose_msg.pose.position.z = jt.points[0].positions[2];
		_bypass_target_pub.publish(pose_msg);
	}
	else if (mission_type == 5)
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			home_transformation.translation().x(),
			home_transformation.translation().y(), 
			_takeoff_height));
	}
	else if (mission_type == 6)
	{
		wp_pos_vector.push_back(Eigen::Vector3d(
			global_nwu_pose.translation().x(),
			global_nwu_pose.translation().y(), 
			home_transformation.translation().z()));
	}

	return mission_type;
}

bool pbtm_class::update_get_command_by_time()
{
	// Bspline is updated before we reach here;
	// bs_control_points = cp;

	time_point<std::chrono::system_clock> now_time = 
		system_clock::now();

	double rel_now_time = duration<double>(now_time - stime).count();
	if ((timespan[1] - rel_now_time) < 0)
	{
		std::cout << "[pbtm.cpp] rel_now_time is outside of timespan[1]" << KNRM << std::endl;
		return false;
	}

	bspline_trajectory::bs_pva_state_3d pva3;
	pva3 = bsu.get_single_bspline_3d(
		_order, timespan, control_points, rel_now_time);

	cmd_nwu.pos = pva3.pos[0];
	cmd_nwu.t = rel_now_time;
	
	// if (!pva3.vel.empty())
	cmd_nwu.vel = pva3.vel[0];
	double _norm = sqrt(pow(cmd_nwu.vel.x(),2) + pow(cmd_nwu.vel.y(),2));
	double _norm_x = cmd_nwu.vel.x() / _norm;
	double _norm_y = cmd_nwu.vel.y() / _norm;

	if (!pva3.acc.empty())
		cmd_nwu.acc = pva3.acc[0];
	
	// If velocity is too low, then any noise will cause the yaw to fluctuate
	// Restrict the yaw if velocity is too low
	if (cmd_nwu.vel.norm() >= 0.15)
		last_yaw = atan2(_norm_y,_norm_x);

	cmd_nwu.q = 
		calculate_quadcopter_orientation(cmd_nwu.acc, last_yaw);

	return true;
}

void pbtm_class::visualize_log_path()
{
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";

	for (int i = 0; i < control_points.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "world";
		pose.pose.position = vector_to_point(control_points[i]);
		path.poses.push_back(pose);
	}

	_log_path_pub.publish(path);

}

/** @brief  dynamic reconfigure callback*/
void pbtm_class::dynamicReconfigureCallback(pbtm::PbtmConfig &config, uint32_t level)
{
	// ROS_INFO("%sreconfiguration request received.%s\n", KBLU, KNRM);
	ROS_INFO("\033[40;37m reconfiguration request received. \033[0m\n");
	// printf("%s[main.cpp] FCU connected! %s\n", KBLU, KNRM);
	if (max_fb_acc_ != config.max_acc)
	{
		max_fb_acc_ = config.max_acc;
		// ROS_INFO("%sReconfigure request : max_acc = %.2f %s\n", KYEL,config.max_acc,KNRM);
		ROS_INFO("\033[40;37m Reconfigure request : max_acc = %.2f \033[0m\n", config.max_acc);
	}

	else if (Kpos_x_ != config.Kpos_x_) 
	{
    	Kpos_x_ = config.Kpos_x_;
    	// ROS_INFO("%s Reconfigure request : Kpos_x_  = %.2f %s\n", KYEL, config.Kpos_x_, KNRM);
		ROS_INFO("\033[40;37m Reconfigure request : Kpos_x_  = %.2f \033[0m\n", config.Kpos_x_);
  	} 
	
	else if (Kpos_y_ != config.Kpos_y_) 
	{
    	Kpos_y_ = config.Kpos_y_;
    	ROS_INFO("\033[40;37m Reconfigure request : Kpos_y_  = %.2f \033[0m\n", config.Kpos_y_);
	} 
	
	else if (Kpos_z_ != config.Kpos_z_) 
	{
    	Kpos_z_ = config.Kpos_z_;
    	ROS_INFO("\033[40;37m Reconfigure request : Kpos_z_  = %.2f \033[0m\n", config.Kpos_z_);
	} 
	
	else if (Kvel_x_ != config.Kvel_x_) 
	{
    	Kvel_x_ = config.Kvel_x_;
    	ROS_INFO("\033[40;37m Reconfigure request : Kvel_x_  = %.2f \033[0m\n", config.Kvel_x_);
  	} 
	
	else if (Kvel_y_ != config.Kvel_y_) 
	{
    	Kvel_y_ = config.Kvel_y_;
    	ROS_INFO("\033[40;37m Reconfigure request : Kvel_y_ =%.2f \033[0m\n", config.Kvel_y_);
  	} 
	
	else if (Kvel_z_ != config.Kvel_z_) 
	{
    	Kvel_z_ = config.Kvel_z_;
    	ROS_INFO("\033[40;37m Reconfigure request : Kvel_z_  = %.2f \033[0m\n", config.Kvel_z_);
  	} 
	
	else if (norm_thrust_const_ != config.norm_thrust_const_) 
	{
    	norm_thrust_const_ = config.norm_thrust_const_;
    	ROS_INFO("\033[40;37m Reconfigure request : norm_thrust_const_  = %.2f \033[0m\n", config.norm_thrust_const_);
  	} 
	
	else if (norm_thrust_offset_ != config.norm_thrust_offset_) 
	{
    	norm_thrust_offset_ = config.norm_thrust_offset_;
    	ROS_INFO("\033[40;37m Reconfigure request : norm_thrust_offset_  = %.2f \033[0m\n", config.norm_thrust_offset_);
	}

  	Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  	Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
	pos_ctrl->setPosGains(Kpos_);
	pos_ctrl->setVelGains(Kvel_);
}