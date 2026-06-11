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
 */
#pragma once

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include "matrix/Matrix.hpp"
#include "matrix/Quaternion.hpp"
#include "matrix/Euler.hpp"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>
#include <vector>

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

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl();

	void arm();
	void disarm();
	void setState(NodeState state) {_state = state;}

	void key_input();
private:
	rclcpp::TimerBase::SharedPtr timer_;

	bool _first_odom{false};
	bool _first_traj{false};
    
    milliseconds _timer_period{10ms};
	float _timer_freq{100.0f};

	CARTESIAN_PLANNER _trajectory{_timer_freq};
    void firstTraj();
	void takeoffTraj(float alt);
	void startTraj(matrix::Vector3f pos, float roll, float pitch, float yaw, double d);
		
    rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
	
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    matrix::Quaternionf _attitude{};
	matrix::Vector3f _position{};
	matrix::Vector3f _last_pos_sp{};
	matrix::Quaternionf _last_att_sp{};

    std::vector<double> _traj_points;
	bool _traj_present = false;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    NodeState _state;
	TrajectorySetpoint _current_setpoint_msg;
	matrix::Vector3f _goal{};
	matrix::Vector3f _starting_point{};
	float _starting_yaw{};

	geometry_msgs::msg::PoseStamped _x;
	geometry_msgs::msg::TwistStamped _xd;
	geometry_msgs::msg::AccelStamped _xdd;
};
