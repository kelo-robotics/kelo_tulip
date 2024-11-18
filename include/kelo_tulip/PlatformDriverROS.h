/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Walter Nowak
 * Sebastian Blumenthal
 * Dharmin Bakaraniya
 * Nico Huebel
 * Arthur Ketels
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef KELOTULIP_PLATFORMDRIVERROS_H
#define KELOTULIP_PLATFORMDRIVERROS_H

#include <memory>

#include "kelo_tulip/EtherCATModuleROS.h"
#include "kelo_tulip/PlatformDriver.h"
#include "kelo_tulip/msg/kelo_drives_input.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using std::placeholders::_1;

namespace kelo {

class PlatformDriverROS : public EtherCATModuleROS {
public:
	//! Default constructor.
	PlatformDriverROS();
	
	//! Default destructor.
	virtual ~PlatformDriverROS();

	//! Initialize this module, must be overridden.
	//! Returns true if module could be successfully initialized.
	virtual bool init(rclcpp::Node::SharedPtr nh, std::string configPrefix);

	//! Function that is continously called by ROS main loop to publish or process data.
	//! Returns true if module can continue to run.
	virtual bool step();

	//! Return the type of this module as string.
	virtual std::string getType();

	//! Return internal EtherCAT module, must be overridden.
	virtual EtherCATModule* getEtherCATModule();

protected:
	virtual kelo::PlatformDriver* createDriver();
	
	void readWheelModels();
	void readWheelConfig();
	void checkAndPublishSmartWheelStatus();
	
	void initializeEncoderValue();
	void calculateRobotVelocity(double& vx, double& vy, double& va, double& encDisplacement);
	void calculateRobotPose(double vx, double vy, double va);
	void publishOdometry(double vx, double vy, double va);
	void createOdomToBaseLinkTransform(geometry_msgs::msg::TransformStamped& odom_trans);

	void publishProcessDataInput();
	void publishBattery();
	void publishIMU();

	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
	void currentMaxCallback(const std_msgs::msg::Float32::SharedPtr msg) const;
	void resetCallback(const std_msgs::msg::Empty::SharedPtr msg) const;
	void enableCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;

	virtual void joyCallbackImpl(const sensor_msgs::msg::Joy::SharedPtr joy);

	kelo::PlatformDriver* driver;
	std::vector<kelo::WheelConfig> wheelConfigs;
	std::vector<kelo::WheelData> wheelData;
	std::vector<kelo::EtherCATModuleROS*> wheelModules;
	std::map<std::string, kelo::WheelModel> wheelModels;

	rclcpp::Publisher<kelo_tulip::msg::KeloDrivesInput>::SharedPtr processDataInputPublisher;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr odomInitializedPublisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mileagePublisher;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
	//rclcpp::Publisher<>::SharedPtr valuesPublisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPublisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr errorPublisher;
	//rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr timestampPublisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr statusPublisher;

	std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
	
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSubscriber;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enableSubscriber;
		
	double s_w; //caster offset of a smartWheel
	double d_w; //distance between the left and the right wheel
	double s_d_ratio;	
	double r_w; //the radius of the wheel

	int nWheels;

	bool useJoy;
	bool debugMode;
	bool activeByJoypad;

	double currentMax;

	double joyVlinMax;
	double joyVaMax;
	double joyScale;
	std::vector<float> prev_axes;

	std::vector<double> prev_left_enc;
	std::vector<double> prev_right_enc;
	double odomx;
	double odomy;
	double odoma;
	
	rclcpp::Node::SharedPtr nh;
};

} // namespace kelp

#endif // KELOTULIP_PLATFORMDRIVERROS_H
