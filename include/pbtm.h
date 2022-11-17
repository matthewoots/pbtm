/*
* pbtm.h
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
#ifndef PBTM_H
#define PBTM_H

#include <string>
#include <mutex>
#include <iostream>
#include <chrono>
#include <ctime>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <random>

#include <nav_msgs/Path.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <dynamic_reconfigure/server.h>
#include <pbtm/PbtmConfig.h>

#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

#include "bspline_utils.hpp"
#include "offb_ctrl.h"


#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace trajectory;

class pbtm_class
{
    private:

        /** @brief VehicleTask determines the task description/mode of the agent **/
        enum VehicleTask
        {
            kIdle,
            kTakeOff,
            kHover,
            kMission,
            kBypass,
            kHome,
            kLand
        };

        /** @brief TaskToString interprets the input mission_value **/
        const std::string TaskToString(int v)
        {
            switch (v)
            {
                case kIdle:   return "IDLE";
                case kTakeOff:   return "TAKEOFF";
                case kHover: return "HOVER";
                case kMission:   return "MISSION";
                case kBypass:   return "BYPASS";
                case kHome:   return "HOME";
                case kLand: return "LAND";
                default:      return "[Unknown Task]";
            }
        }

        /** @brief state_command structure to unify command values to setpoint_raw local **/
        struct state_command
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            Eigen::Quaterniond q;
            double t;
        };
        pbtm_class::state_command cmd_nwu;

        /** @brief classes from libbspline packages (functions) that are used in this package **/
        bspline_trajectory bsu;
        common_trajectory_tool ctt;
        offboard_controller::OffbCtrl* pos_ctrl = nullptr;

        /** @brief Service clients **/
        ros::ServiceClient arming_client, set_mode_client; 
        /** @brief Handles mavros states and arming mode **/
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::State uav_current_state;

        ros::NodeHandle _nh;

        /** @brief Subscribers **/
        ros::Subscriber _pos_sub, _vel_sub, _waypoint_sub, _state_sub, _bypass_sub, _flat_ref_enu_sub, _batt_sub;
        /** @brief Publishers **/
        ros::Publisher _pose_nwu_pub, _local_pos_raw_pub, _log_path_pub, _bypass_target_pub, _att_rate_pub, _ref_enu_pub;

        /** @brief Timers **/
        ros::Timer _drone_timer;

        /** @brief Saving ros::Time information of various data **/
        ros::Time _last_pose_time, _prev_bypass_msg_time, bypass_timeout_start_time;

        /** Offboard enabled means that offboard control is active and also uav is armed **/
        bool _offboard_enabled = false;
        bool _setup = false;
        bool _set_takeoff_land_orientation;
        bool _send_acceleration_setpoint;
        double takeoff_land_velocity;
        double _bypass_timeout;
        int uav_id, uav_task;
        std::string _id;
        double _send_command_interval, _send_command_rate;
        double _timeout, _nwu_yaw_offset, last_yaw, _takeoff_height;
        Eigen::Vector3d _start_global_nwu; // origin in the global frame

        /** @brief Bspline parameters **/
        int _knot_division, _knot_size;
        double _order, _max_velocity;
        double _knot_interval, _duration;
        Eigen::Quaterniond _takeoff_land_orientation;

        time_point<std::chrono::system_clock> stime; // start time for bspline server in time_t
        vector<double> timespan;

        /** @brief mutexes **/
        std::mutex send_command_mutex;
        std::mutex pose_mutex;
        std::mutex waypoint_command_mutex;

        /** @brief Important transformations that handle global and local conversions **/
        Eigen::Affine3d current_transform_enu, global_nwu_pose, home_transformation;
        Eigen::Affine3d global_to_local_t, local_to_global_t;

        Eigen::Vector3d uav_local_vel_enu; // uav local velocity in ENU
        Eigen::Vector3d uav_local_pos_enu; // uav local position in ENU
        Eigen::Vector3d uav_att_rate; // uav attitude rate
        Eigen::Vector3d targetPos_, targetVel_, targetAcc_;


        double battery_volt;
        // Control gains (position, velocity, drag)
        Eigen::Vector3d Kpos_, Kvel_, D_;
        Eigen::Vector3d gravity_;
        double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
        std::vector<double> thrust_coeff_, volt_coeff_, thrust_original_;

        bool px4Ctrl;
        bool voltage_compensation_;
        bool using_yawTgt;
        double yaw_ref;
        double max_fb_acc_;
        double attctrl_tau_;
        double norm_thrust_offset_;
        double norm_thrust_const_;
        double mass_; // mass of platform

        double m_a_, m_b_, m_c_, volt_k_, volt_b_;
        double throttle_offset_, throttle_limit_;

        Eigen::Vector4d q_des, uav_attitude_q;
        Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}

        /** @brief For path visualization **/
        nav_msgs::Path path;

        vector<Eigen::Vector3d> wp_pos_vector;
        vector<Eigen::Vector3d> control_points;
        vector<double> height_list;

        /** @brief Conversion (rotation) from enu to nwu in the form of w, x, y, z **/
        Quaterniond enu_to_nwu() {return Quaterniond(0.7073883, 0, 0, 0.7068252);}
    
    public:

        /** @brief Constructor **/
        pbtm_class(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            _nh.param<std::string>("agent_id", _id, "drone0");
            _nh.param<double>("send_command_rate", _send_command_rate, 1.0);
            _nh.param<double>("timeout", _timeout, 0.5);
            _nh.param<double>("takeoff_height", _takeoff_height, 1.0);

            std::vector<double> position_list;
            _nh.getParam("global_start_position", position_list);
            _start_global_nwu.x() = position_list[0];
            _start_global_nwu.y() = position_list[1];
            _start_global_nwu.z() = position_list[2];

            // height min followed by height max
            _nh.getParam("height_range", height_list);
            _nh.param<double>("yaw_offset_rad", _nwu_yaw_offset, 0.0);

            /** @brief Bspline parameters **/
            _nh.param<double>("order", _order, 1.0);
            _nh.param<double>("max_velocity", _max_velocity, 1.0);
            _nh.param<int>("knot_division", _knot_division, 1);

            /** @brief Other parameters **/
            _nh.param<double>("bypass_timeout", _bypass_timeout, 2.0);
            _nh.param<double>("takeoff_land_velocity", takeoff_land_velocity, 0.2);

            /** @brief Control gains**/
            _nh.param<double>("gains/p_x", Kpos_x_, 8.0);
            _nh.param<double>("gains/p_y", Kpos_y_, 8.0);
            _nh.param<double>("gains/p_z", Kpos_z_, 10.0);
            _nh.param<double>("gains/v_x", Kvel_x_, 1.5);
            _nh.param<double>("gains/v_y", Kvel_y_, 1.5);
            _nh.param<double>("gains/v_z", Kvel_z_, 3.3);

            _nh.param<bool>("send_acceleration_setpoint", _send_acceleration_setpoint, false);
            _nh.param<bool>("use_px4ctrl", px4Ctrl, true);
            _nh.param<bool>("use_yawTarget", using_yawTgt, false);
            _nh.param<double>("max_acc", max_fb_acc_, 9.0);

            _nh.param<double>("attctrl_constant", attctrl_tau_, 0.1);
            _nh.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);
            _nh.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);
            _nh.param<bool>("voltage_compensation", voltage_compensation_, false);

            _nh.param<double>("m_a", m_a_, 202.33);
            _nh.param<double>("m_b", m_b_, 145.56);
            _nh.param<double>("m_c", m_c_, -8.0219);

            _nh.param<double>("volt_k", volt_k_, -0.1088);
            _nh.param<double>("volt_b", volt_b_, 2.1964);

            _nh.param<double>("throttle_offset", throttle_offset_, 0.06);
            _nh.param<double>("throttle_limit", throttle_limit_, 1);

            _nh.param<double>("mass", mass_, 195.5);

            Kpos_ = Vector3d(-Kpos_x_, -Kpos_y_, -Kpos_z_);
            Kvel_ = Vector3d(-Kvel_x_, -Kvel_y_, -Kvel_z_);

            thrust_coeff_ = {m_a_, m_b_, m_c_};
            volt_coeff_ = {volt_k_, volt_b_};
            thrust_original_ = {norm_thrust_const_, norm_thrust_offset_};

            std::cout << "Position gains are: " << Kpos_.transpose() << std::endl;
            std::cout << "Velocity gains are: " << Kvel_.transpose() << std::endl;

            uav_local_vel_enu = Vector3d(0.0, 0.0, 0.0);
            uav_att_rate = Vector3d(0.0, 0.0, 0.0);
            gravity_ = Vector3d(0.0, 0.0, -9.8);

            _send_command_interval = 1 / _send_command_rate;

            // Reset uav task to idle
            uav_task = 0;
            // Reset commanded state
            cmd_nwu.pos = Eigen::Vector3d::Zero();
            cmd_nwu.vel = Eigen::Vector3d::Zero();
            cmd_nwu.acc = Eigen::Vector3d::Zero();
            cmd_nwu.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); 

            // For inclusion of yaw offset
            Eigen::Quaterniond q_corrected;
            q_corrected = AngleAxisd(0, Eigen::Vector3d::UnitX())
                * AngleAxisd(0, Eigen::Vector3d::UnitY())
                * AngleAxisd(_nwu_yaw_offset, Eigen::Vector3d::UnitZ());
            // Setup local_to_global transform
            local_to_global_t = Affine3d::Identity(); 
            local_to_global_t.translate(_start_global_nwu);
            local_to_global_t.rotate(q_corrected.inverse());
            local_to_global_t.rotate(enu_to_nwu().inverse());

            // Setup global_to_local transform
            global_to_local_t = Affine3d::Identity(); 
            global_to_local_t.rotate(enu_to_nwu());
            global_to_local_t.rotate(q_corrected);
            global_to_local_t.translate(-_start_global_nwu);

            /** @brief Get the uav id in int **/
            std::string copy_id = _id; 
            std::string uav_id_char = copy_id.erase(0,5); // removes first 5 character
            uav_id = stoi(uav_id_char);

            /* ------------ Subscribers ------------ */
            /** @brief Get Mavros State of PX4 */
            _state_sub = _nh.subscribe<mavros_msgs::State>(
                "/" + _id + "/mavros/state", 10, boost::bind(&pbtm_class::uavStateCallBack, this, _1));
            /** @brief Subscriber that receives local position via mavros */
            _pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
                "/" + _id + "/mavros/local_position/pose", 20, &pbtm_class::pose_callback, this);
            /** @brief Subscriber that receives local velocity via mavros */
            _vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>(
                "/" + _id + "/mavros/local_position/velocity_local", 1, &pbtm_class::vel_callback, this);
            /** @brief Subscriber that receives waypoint information from user */
            _waypoint_sub = _nh.subscribe<trajectory_msgs::JointTrajectory>(
                "/trajectory/points", 20, &pbtm_class::waypoint_command_callback, this);
            _bypass_sub = _nh.subscribe<mavros_msgs::PositionTarget>(
                "/" + _id + "/bypass", 20, &pbtm_class::bypass_callback, this);

            _batt_sub = _nh.subscribe(
                "/" + _id + "/mavros/battery", 1, &pbtm_class::battery_callback, this);
            

            /* ------------ Publishers ------------ */
            /** @brief Publisher that publishes control raw setpoints via mavros */
            _local_pos_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
                "/" + _id + "/mavros/setpoint_raw/local", 20);
            /** @brief Publisher that publishes global_pose in nwu frame */
            _pose_nwu_pub = _nh.advertise<geometry_msgs::PoseStamped>(
                "/" + _id + "/uav/nwu", 20);
            _log_path_pub = _nh.advertise<nav_msgs::Path>(
                "/" + _id + "/uav/log_path", 10, true);
            _bypass_target_pub = _nh.advertise<geometry_msgs::PoseStamped>(
                "/" + _id + "/goal", 10, true);

            /**@brief Publisher attitude rate (roll, pitch, yaw, thrust)*/
            _att_rate_pub = _nh.advertise<mavros_msgs::AttitudeTarget>(
                "/" + _id + "/mavros/setpoint_raw/attitude", 1, true
            );

            _ref_enu_pub = _nh.advertise<mavros_msgs::PositionTarget>(
                "/" + _id + "/cmd_enu", 1, true
            );



            /* ------------ Timers ------------ */
            /** @brief Timer that handles drone state at each time frame */
            _drone_timer = _nh.createTimer(ros::Duration(
                _send_command_interval), &pbtm_class::drone_timer, this, false, false);


            /* ------------ Service Clients ------------ */
            /** @brief Service Client that handles arming in Mavros */
            arming_client = _nh.serviceClient<mavros_msgs::CommandBool>(
                "/" + _id + "/mavros/cmd/arming");
            /** @brief Service Client that handles mode switching in Mavros */
            set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(
                "/" + _id + "/mavros/set_mode");

            printf("[%sdrone%d%s pbtm.h] global_start_pose [%s%.2lf %.2lf %.2lf%s]! \n", 
                KGRN, uav_id, KNRM,
                KBLU, _start_global_nwu(0), _start_global_nwu(1), _start_global_nwu(2), KNRM);
            printf("[%sdrone%d%s pbtm.h] height_range [%s%.2lf %.2lf%s]! \n", 
                KGRN, uav_id, KNRM,
                KBLU, height_list[0], height_list[1], KNRM);
            printf("[%sdrone%d%s pbtm.h] yaw_offset_rad [%s%.2lf%s]! \n", 
                KGRN, uav_id, KNRM, KBLU, _nwu_yaw_offset, KNRM);

            printf("%s[drone%d%s pbtm.h] constructed! \n", KGRN, uav_id, KNRM);

            /** @brief offboard position controller class*/
            // OffbCtrl pos_ctrl();

            //if (!px4Ctrl)
            //{
                // std::unique_ptr<offboard_controller::OffbCtrl> pos_ctrl(

                pos_ctrl = new offboard_controller::OffbCtrl(
                    mass_,
                    Kpos_,
                    Kvel_,
                    thrust_coeff_,
                    volt_coeff_,
                    max_fb_acc_,
                    throttle_offset_,
                    throttle_limit_,
                    thrust_original_
                );
            //}


            _drone_timer.start();
        }

        /** @brief Destructor **/
        ~pbtm_class()
        {
            _drone_timer.stop();
        }

        /** @brief check_last_time helps to check whether queried ros::Time satisfy the tolerance margin in the input with the current time **/
        bool check_last_time(double tolerance, ros::Time time);

        /** @brief Main timer (thread) for sending commands and switch modes **/
        void drone_timer(const ros::TimerEvent &);

        /** @brief Pack pbtm_class::state_command cmd_nwu into setpoint_raw/local and publishes it **/
        void send_command();

        /** @brief Initializes the bspline parameters and also sets up the control points **/
        void initialize_bspline_server(double desired_velocity);

        /** @brief To get the command after the Bspline server has started **/
        bool update_get_command_by_time();

        /** @brief Set offboard mode (or tries to set it) **/
        void set_offboard();

        /** @brief stop and hover just sets any last velocity and acceleration command to 0 so that there will be no feedforward **/
        void stop_and_hover();

        /** @brief visualization of the path/trajectory (control points) **/
        void visualize_log_path();

        /** @brief callbacks */
        void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
        void vel_callback(const geometry_msgs::TwistStampedConstPtr& msg);
        void waypoint_command_callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
        void uavStateCallBack(const mavros_msgs::State::ConstPtr &msg);
        void bypass_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
        void battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);

        void dynamicReconfigureCallback(pbtm::PbtmConfig &config, uint32_t level);

        /** @brief Used by controller*/

        double getYawFromVel(const Eigen::Vector3d &vel) {return atan2(vel(1), vel(0));};

        /** @brief common utility functions */
        int joint_trajectory_to_waypoint(trajectory_msgs::JointTrajectory jt);

};


#endif
