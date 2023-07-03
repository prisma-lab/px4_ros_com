/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#pragma once

#include <px4_msgs/msg/tilting_attitude_setpoint.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

// #include <Eigen/Matrix>
// #include <Eigen/Geometry>
#include "matrix/Matrix.hpp"
#include "matrix/Quaternion.hpp"
#include "matrix/Euler.hpp"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>

#include <chrono>
#include <iostream>

#include "Trajectory.hpp"
#include "planner_spline.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum NodeState {
	STOPPED = 0,
	FLYING,
	HOVERING
};


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl();

	/**
	 * @brief Send a command to Arm the vehicle
	 */
	void arm() const;
	/**
	 * @brief Send a command to Disarm the vehicle
	 */
	void disarm() const;

	void setState(NodeState state) {_state = state;}

	void key_input();


private:
	void timer_callback();

	bool _first_odom{false};
	bool _first_traj{false};

	rclcpp::TimerBase::SharedPtr _timer;
	milliseconds _timer_period{10ms};
	float _timer_freq{100.0f};

	CARTESIAN_PLANNER _trajectory{_timer_freq};
	void firstTraj();
	void takeoffTraj(float alt);
	void startTraj(matrix::Vector3f pos, float roll, float pitch, float yaw, double d);

	rclcpp::Publisher<TiltingAttitudeSetpoint>::SharedPtr _tilting_attitude_setpoint_publisher;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;

	std::atomic<uint64_t> _timestamp;   //!< common synced timestamped

	matrix::Quaternionf _attitude{};
	matrix::Vector3f _position{};
	matrix::Vector3f _last_pos_sp{};
	matrix::Quaternionf _last_att_sp{};

	uint64_t _offboard_setpoint_counter;   //!< counter for the number of setpoints sent

	/**
	 * @brief Publish the offboard control mode.
	 *        For this example, only position and altitude controls are active.
	 */
	void publish_offboard_control_mode() const;
	/**
	 * @brief Publish a trajectory setpoint
	 *        For this example, it sends a trajectory setpoint to make the
	 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
	 */
	void publish_trajectory_setpoint() const;
	/**
	 * @brief Publish vehicle commands
	 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	 * @param param1    Command parameter 1
	 * @param param2    Command parameter 2
	 */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0) const;

	NodeState _state;
	TrajectorySetpoint _current_setpoint_msg;
	matrix::Vector3f _goal{};
	matrix::Vector3f _starting_point{};
	float _starting_yaw{};

	geometry_msgs::msg::PoseStamped _x;
	geometry_msgs::msg::TwistStamped _xd;
	geometry_msgs::msg::AccelStamped _xdd;

};