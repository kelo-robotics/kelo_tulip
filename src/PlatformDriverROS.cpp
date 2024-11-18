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


#include "kelo_tulip/PlatformDriverROS.h"

namespace kelo {

PlatformDriverROS::PlatformDriverROS()
	: driver(NULL)
	, odom_broadcaster(nullptr)
{
	s_w = 0.01; //caster offset of a smartWheel
	d_w = 0.0775; //distance between the left and the right wheel
	s_d_ratio = s_w / d_w;	
	r_w = 0.0524; //the radius of the wheel

	nWheels = 0;

	useJoy = false;
	debugMode = false;
	activeByJoypad = false;

	currentMax = 20;

	joyVlinMax = 1.0;
	joyVaMax = 1.0;
	joyScale = 1.0;
	prev_axes.resize(6, 0.0);

	odomx = 0;
	odomy = 0;
	odoma = 0;
}

PlatformDriverROS::~PlatformDriverROS() {
	if (driver)
		delete driver;

	if (odom_broadcaster)
		odom_broadcaster.reset();
		//delete odom_broadcaster;
}

bool PlatformDriverROS::init(rclcpp::Node::SharedPtr nh, std::string configPrefix) {
	this->nh = nh;
	
	nh->declare_parameter("num_wheels", 0);
	nh->declare_parameter("current_stop", 20.0);
	nh->declare_parameter("current_drive", 20.0);
	nh->declare_parameter("current_max", 20.0);
	nh->declare_parameter("vlin_max", 1.0);
	nh->declare_parameter("va_max", 1.0);
	nh->declare_parameter("vlin_acc_max", 0.5);
	nh->declare_parameter("vlin_dec_max", 0.8); 
	nh->declare_parameter("va_acc_max", 0.5); 
	nh->declare_parameter("va_dec_max", 0.8);
	nh->declare_parameter("angle_acc_max", 0.8);
	nh->declare_parameter("joy_vlin_max", 1.0); 
	nh->declare_parameter("joy_va_max", 1.0); 
	nh->declare_parameter("joy_scale", 1.0);
	nh->declare_parameter("active_by_joypad", false);

	rclcpp::Parameter num_wheels;
	if (!nh->get_parameter("num_wheels", num_wheels)) {
		RCLCPP_ERROR(nh->get_logger(), "Missing number of wheels in config file");
		return -1;
	}
	nWheels = num_wheels.as_int();

	if (nWheels < 0) {
		RCLCPP_ERROR(nh->get_logger(), "Invalid number of wheels in config file");
		return -1;
	}

	wheelConfigs.resize(nWheels);
	kelo::WheelData data = {};
	data.enable = true;
	data.error = false;
	data.errorTimestamp = false;
	wheelData.resize(nWheels, data);

	// read all wheel configs
	readWheelModels();
	readWheelConfig();

	driver = createDriver();

	// set driver control parameters		
	rclcpp::Parameter x;
	if (nh->get_parameter("current_stop", x))
		driver->setCurrentStop(x.as_double());
	if (nh->get_parameter("current_drive", x))
		driver->setCurrentDrive(x.as_double());
	if (nh->get_parameter("current_max", x))
		currentMax = x.as_double();
		
	if (nh->get_parameter("vlin_max", x))
		driver->setMaxvlin(x.as_double());
	if (nh->get_parameter("va_max", x))
		driver->setMaxva(x.as_double());
	if (nh->get_parameter("vlin_acc_max", x))
		driver->setMaxvlinacc(x.as_double());
	if (nh->get_parameter("vlin_dec_max", x))
		driver->setMaxvlindec(x.as_double());
	if (nh->get_parameter("angle_acc_max", x))
		driver->setMaxangleacc(x.as_double());
	if (nh->get_parameter("va_acc_max", x))
		driver->setMaxvaacc(x.as_double());
	if (nh->get_parameter("va_dec_max", x))
		driver->setMaxvadec(x.as_double());

	joyVlinMax = driver->getMaxvlin();
	joyVaMax = driver->getMaxva();
	if (nh->get_parameter("joy_vlin_max", x))
		joyVlinMax = x.as_double();
	if (nh->get_parameter("joy_va_max", x))
		joyVaMax = x.as_double();
	if (nh->get_parameter("joy_scale", x))
		if (x.as_double() > 0 && x.as_double() <= 1.0)
			joyScale = x.as_double();

	rclcpp::Parameter b;
	if (nh->get_parameter("active_by_joypad", b))
		activeByJoypad = b.as_bool();
	if (!activeByJoypad)
		driver->setCanChangeActive();
		
	odomPublisher = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
	odomInitializedPublisher = nh->create_publisher<std_msgs::msg::Empty>("/odom_initialized", 10);
//	timestampPublisher = nh->create_publisher<std_msgs::msg::UInt64MultiArray>("timestamp", 10);
	imuPublisher = nh->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
	processDataInputPublisher = nh->create_publisher<kelo_tulip::msg::KeloDrivesInput>("~/wheels_input", 10);
	batteryPublisher = nh->create_publisher<std_msgs::msg::Float32>("~/battery", 10);
	errorPublisher = nh->create_publisher<std_msgs::msg::Int32>("~/error", 10);
	statusPublisher = nh->create_publisher<std_msgs::msg::Int32>("~/status", 10);
	joySubscriber = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, std::bind(&PlatformDriverROS::joyCallback, this, std::placeholders::_1));
	cmdVelSubscriber = nh->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 5, std::bind(&PlatformDriverROS::cmdVelCallback, this, std::placeholders::_1));
	resetSubscriber = nh->create_subscription<std_msgs::msg::Empty>("reset", 1, std::bind(&PlatformDriverROS::resetCallback, this, std::placeholders::_1));
	enableSubscriber = nh->create_subscription<std_msgs::msg::Int32MultiArray>("wheels_enable", 10, std::bind(&PlatformDriverROS::enableCallback, this, std::placeholders::_1));
	
//	ros::Subscriber currentMaxSubscriber = nh.subscribe("current_max", 1, currentMaxCallback);
	odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nh);
	
	initializeEncoderValue();
	
	
	
	return true;
}

bool PlatformDriverROS::step() {
	checkAndPublishSmartWheelStatus();

	//calculate robot velocity
	double vx, vy, va, encDisplacement;
	calculateRobotVelocity(vx, vy, va, encDisplacement);

	//calculate robot displacement and current pose
	calculateRobotPose(vx, vy, va);
		
	//publish the odometry
	publishOdometry(vx, vy, va);

	//broadcast odom-base_link transform
	geometry_msgs::msg::TransformStamped odom_trans;
	createOdomToBaseLinkTransform(odom_trans);
	odom_broadcaster->sendTransform(odom_trans);
	
/*
		//publish smartwheel values
		std_msgs::msg::float64_multi_array processDataValues;
		for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
			addToWheelDataMsg(processDataValues, driver->getWheelData(i));
			addToProcessDataMsg(processDataValues, driver->getProcessData(wheelConfigs[i].ethercatNumber));
			processDataValues.data.push_back(driver->getCurrentDrive());
			processDataValues.data.push_back(driver->getThreadPhase());
		}
		valuesPublisher->publish(processDataValues);
*/

	publishProcessDataInput();
	publishBattery();

	//publish IMU data
	publishIMU();

	return true;
}

std::string PlatformDriverROS::getType() {
	return "platform_driver";
}

EtherCATModule* PlatformDriverROS::getEtherCATModule() {
	return driver;
}

kelo::PlatformDriver* PlatformDriverROS::createDriver() {
	return new kelo::PlatformDriver(wheelConfigs, wheelData);
}

void PlatformDriverROS::readWheelModels() {
	nh->declare_parameter("wheel_models.list", std::vector<std::string>{});
	rclcpp::Parameter list = nh->get_parameter("wheel_models.list");
	std::vector<std::string> parameterList = list.as_string_array();

	for (unsigned int i = 0; i < parameterList.size(); i++) {
		std::string name = parameterList[i];
		std::string prefix = "wheel_models." + name + ".";
		nh->declare_parameter(prefix + "active", true);
		nh->declare_parameter(prefix + "diameter", 0.105);
		nh->declare_parameter(prefix + "width", 0.040);
		nh->declare_parameter(prefix + "casteroffset", 0.010);
		nh->declare_parameter(prefix + "wheeldistance", 0.08);
		nh->declare_parameter(prefix + "canPivot", true);
		nh->declare_parameter(prefix + "velocitylimit", 100.0);
		nh->declare_parameter(prefix + "currentlimit", 10.0);

		WheelModel wm;
		wm.name = name;
		wm.active = nh->get_parameter(prefix + "active").as_bool();
		wm.diameter = nh->get_parameter(prefix + "diameter").as_double();
		wm.width = nh->get_parameter(prefix + "width").as_double();
		wm.casteroffset = nh->get_parameter(prefix + "casteroffset").as_double();
		wm.wheeldistance = nh->get_parameter(prefix + "wheeldistance").as_double();
		wm.canPivot = nh->get_parameter(prefix + "canPivot").as_bool();
		wm.velocitylimit = nh->get_parameter(prefix + "velocitylimit").as_double();
		wm.currentlimit = nh->get_parameter(prefix + "currentlimit").as_double();
		wheelModels[name] = wm;
	}
	
	//XmlRpc::XmlRpcValue xmllist;
	//nh.getParam("wheel_models", xmllist);
	//for (XmlRpc::XmlRpcValue::iterator it = xmllist.begin(); it != xmllist.end(); ++it) {
		//std::string name = it->first;
		//std::string prefix = "wheel_models/" + name + "/";
		//WheelModel wm;
		//wm.name = name;
		//nh.getParam(prefix + "active", wm.active);
		//nh.getParam(prefix + "diameter", wm.diameter);
		//nh.getParam(prefix + "width", wm.width);
		//nh.getParam(prefix + "casteroffset", wm.casteroffset);
		//nh.getParam(prefix + "wheeldistance", wm.wheeldistance);
		//nh.getParam(prefix + "can_pivot", wm.canPivot);
		//nh.getParam(prefix + "velocitylimit", wm.velocitylimit);
		//nh.getParam(prefix + "currentlimit", wm.currentlimit);
		//wheelModels[name] = wm;
	//}
}

void PlatformDriverROS::readWheelConfig() {
	for (int i = 0; i < nWheels; i++) {
		std::stringstream ssGroupName;
		ssGroupName << "wheel" << i;
		std::string groupName = ssGroupName.str();
		nh->declare_parameter(groupName + ".ethercat_number", 0);
		nh->declare_parameter(groupName + ".x", 0.0);
		nh->declare_parameter(groupName + ".y", 0.0);
		nh->declare_parameter(groupName + ".a", 0.0);

		kelo::WheelConfig config;
		config.enable = true;
		config.reverseVelocity = true;
		rclcpp::Parameter ecatNr, wheelx, wheely, wheela;
		bool ok =		
		     nh->get_parameter(groupName + ".ethercat_number", ecatNr)
		  && nh->get_parameter(groupName + ".x", wheelx)
			&& nh->get_parameter(groupName + ".y", wheely)
			&& nh->get_parameter(groupName + ".a", wheela);
		config.ethercatNumber = ecatNr.as_int();
		config.x = wheelx.as_double();
		config.y = wheely.as_double();
		config.a = wheela.as_double();

		rclcpp::Parameter reverseVelocity;
		if (nh->get_parameter(groupName + ".reverse_velocity", reverseVelocity))
			config.reverseVelocity = (reverseVelocity.as_int() != 0);

		if (!ok)
			RCLCPP_WARN(nh->get_logger(), "Missing config value for wheel %d", i);

		// copy complete model data if provided
		rclcpp::Parameter model;
		if (nh->get_parameter(groupName + ".model", model)) {
			if (wheelModels.count(model.as_string()) > 0) {
				config.model = wheelModels[model.as_string()];
			} else {
				RCLCPP_WARN(nh->get_logger(), "Unknown wheel model: %s", model.value_to_string().c_str());
			}
		}

		// enable separate values for this wheel
		rclcpp::Parameter x;
		if (nh->get_parameter(groupName + ".wheel_distance", x))
			config.model.wheeldistance = x.as_double();
		if (nh->get_parameter(groupName + ".diameter", x))
			config.model.diameter = x.as_double();

		wheelConfigs[i] = config;
	}
}

void PlatformDriverROS::checkAndPublishSmartWheelStatus() {
	int status = driver->getDriverStatus();
	// int state = (status & 0x000000ff);
	int error = (status & 0xffffff00);
		
	std_msgs::msg::Int32 statusMsg;
	statusMsg.data = status;
	statusPublisher->publish(statusMsg);

	std_msgs::msg::Int32 errorMsg;
	if (error) {
		// TODO correct
		//stop navigation and start debug mode. Robot can only be moved with joystick
		debugMode = true;
		errorMsg.data = status;
		errorPublisher->publish(errorMsg);
		statusPublisher->publish(statusMsg);
	} else {
		if (debugMode) {
			debugMode = false;
			errorMsg.data = 0;
			errorPublisher->publish(errorMsg);
		}
	}
}

double norm(double x) {
	const double TWO_PI = 2.0 * M_PI;
	while (x < -M_PI) {
		x += TWO_PI;
	}
	while (x > M_PI) {
		x -= TWO_PI;
	}

	return x;
}

void PlatformDriverROS::initializeEncoderValue() {
	prev_left_enc.resize(nWheels, 0);
	prev_right_enc.resize(nWheels, 0);
	for (int i=0; i<nWheels; i++) {
		std::vector<double> encoderValueInit = driver->getEncoderValue(i);
		prev_left_enc[i] = encoderValueInit[0];
		prev_right_enc[i] = encoderValueInit[1];
	}
}

void PlatformDriverROS::calculateRobotVelocity(double& vx, double& vy, double& va, double& encDisplacement) {
	double dt = 0.05;
	
	//initialize the variables
	vx = 0;
	vy = 0;
	va = 0;
	encDisplacement = 0;
	
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = driver->getWheelProcessData(i);
		std::vector<double> encoderValue = driver->getEncoderValue(i);
		double wl = (encoderValue[0] - prev_left_enc[i]) / dt;
		double wr = -(encoderValue[1] - prev_right_enc[i]) / dt;
		encDisplacement += fabs(encoderValue[0] - prev_left_enc[i]) + fabs(encoderValue[1] - prev_right_enc[i]);
		prev_left_enc[i] = encoderValue[0];
		prev_right_enc[i] = encoderValue[1];
		double theta = norm(swData->encoder_pivot - wheelConfigs[i].a); // encoder_offset can be obtained from the yaml file or smartWheelDriver class
//std::cout << "theta " << i << ": " << theta << std::endl;
		if (!wheelConfigs[i].reverseVelocity) {
			vx += r_w * ((wl + wr) * cos(theta)); // + 2 * s_d_ratio * (wl - wr) * sin(theta));
			vy += r_w * ((wl + wr) * sin(theta)); // - 2 * s_d_ratio * (wl - wr) * cos(theta));
		} else {
			vx -= r_w * ((wl + wr) * cos(theta)); // + 2 * s_d_ratio * (wl - wr) * sin(theta));
			vy -= r_w * ((wl + wr) * sin(theta)); // - 2 * s_d_ratio * (wl - wr) * cos(theta));		
		}
		double wangle = atan2(wheelConfigs[i].y, wheelConfigs[i].x);
		double d = sqrt(wheelConfigs[i].x * wheelConfigs[i].x + wheelConfigs[i].y * wheelConfigs[i].y);
		if (!wheelConfigs[i].reverseVelocity) {
			va += r_w * (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) + (wr + wl) * sin(theta - wangle)) / d;
		} else{
			va += r_w * (2 * (wr - wl) * s_d_ratio * cos(theta - wangle) - (wr + wl) * sin(theta - wangle)) / d;
		}			
		//va += r_w * (wr + wl) * sin(theta - wangle) / d;
		//va += 4*swData->gyro_y;
	}
	// averaging the wheel velocity
	vx = vx / nWheels / 2;
	vy = vy / nWheels / 2;
	va = va / nWheels / 2;
}

void PlatformDriverROS::calculateRobotPose(double vx, double vy, double va) {
	double dt = 0.05;
	double dx, dy;
	
	if (fabs(va) > 0.001) {
		double vlin = sqrt(vx * vx + vy * vy);
		double direction = atan2(vy, vx);
		double circleRadius = fabs(vlin / va);
		double sign = 1;
		if (va < 0)
			sign = -1;
		//displacement relative to direction of movement
		double dx_rel = circleRadius * sin (fabs(va) * dt);
		double dy_rel = sign * circleRadius * (1 - cos(fabs(va) * dt));

		//transform displacement to previous robot frame
		dx = dx_rel * cos(direction) - dy_rel * sin(direction);
		dy = dx_rel * sin(direction) + dy_rel * cos(direction);
	}
	else {
		dx = vx * dt;
		dy = vy * dt;
	}
	
	//transform displacement to odom frame
	odomx += dx * cos(odoma) - dy * sin(odoma);
	odomy += dx * sin(odoma) + dy * cos(odoma);
	odoma = norm(odoma + va * dt);
}

void PlatformDriverROS::publishOdometry(double vx, double vy, double va) {
	tf2::Quaternion odom_quat;
	odom_quat.setRPY(0, 0, odoma);
	nav_msgs::msg::Odometry odom;
	odom.header.stamp = nh->get_clock()->now();
	//odom.header.seq = sequence_id++;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.pose.covariance[0] = 1e-3;
	odom.pose.covariance[7] = 1e-3;
	odom.pose.covariance[8] = 0.0;
	odom.pose.covariance[14] = 1e6;
	odom.pose.covariance[21] = 1e6;
	odom.pose.covariance[28] = 1e6;
	odom.pose.covariance[35] = 1e3;
	odom.twist.covariance[0] = 1e-3;
	odom.twist.covariance[7] = 1e-3;
	odom.twist.covariance[8] = 0.0;
	odom.twist.covariance[14] = 1e6;
	odom.twist.covariance[21] = 1e6;
	odom.twist.covariance[28] = 1e6;
	odom.twist.covariance[35] = 1e3;
	odom.pose.pose.position.x = odomx;
	odom.pose.pose.position.y = odomy;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf2::toMsg(odom_quat);	
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = va;
	odomPublisher->publish(odom);
}
		
void PlatformDriverROS::createOdomToBaseLinkTransform(geometry_msgs::msg::TransformStamped& odom_trans) {
	tf2::Quaternion odom_quat;
	odom_quat.setRPY(0, 0, odoma);
	odom_trans.header.stamp = nh->get_clock()->now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = odomx;
	odom_trans.transform.translation.y = odomy;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf2::toMsg(odom_quat);
}

void PlatformDriverROS::publishProcessDataInput() {
	kelo_tulip::msg::KeloDrivesInput msg;
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = driver->getWheelProcessData(i);
		kelo_tulip::msg::KeloDriveInput wheel;
		wheel.status1 = swData->status1;
		wheel.status2 = swData->status2;
		wheel.sensor_ts = swData->sensor_ts;
		wheel.setpoint_ts = swData->setpoint_ts;
		wheel.encoder_1 = swData->encoder_1;
		wheel.velocity_1 = swData->velocity_1;
		wheel.current_1_d = swData->current_1_d;
		wheel.current_1_q = swData->current_1_q;
		wheel.current_1_u = swData->current_1_u;
		wheel.current_1_v = swData->current_1_v;
		wheel.current_1_w = swData->current_1_w;
		wheel.voltage_1 = swData->voltage_1;
		wheel.voltage_1_u = swData->voltage_1_u;
		wheel.voltage_1_v = swData->voltage_1_v;
		wheel.voltage_1_w = swData->voltage_1_w;
		wheel.temperature_1 = swData->temperature_1;
		wheel.encoder_2 = swData->encoder_2;
		wheel.velocity_2 = swData->velocity_2;
		wheel.current_2_d = swData->current_2_d;
		wheel.current_2_q = swData->current_2_q;
		wheel.current_2_u = swData->current_2_u;
		wheel.current_2_v = swData->current_2_v;
		wheel.current_2_w = swData->current_2_w;
		wheel.voltage_2 = swData->voltage_2;
		wheel.voltage_2_u = swData->voltage_2_u;
		wheel.voltage_2_v = swData->voltage_2_v;
		wheel.voltage_2_w = swData->voltage_2_w;
		wheel.temperature_2 = swData->temperature_2;
		wheel.encoder_pivot = swData->encoder_pivot;
		wheel.velocity_pivot = swData->velocity_pivot;
		wheel.voltage_bus = swData->voltage_bus;
		wheel.imu_ts = swData->imu_ts;
		wheel.accel_x = swData->accel_x;
		wheel.accel_y = swData->accel_y;
		wheel.accel_z = swData->accel_z;
		wheel.gyro_x = swData->gyro_x;
		wheel.gyro_y = swData->gyro_y;
		wheel.gyro_z = swData->gyro_z;
		wheel.temperature_imu = swData->temperature_imu;
		wheel.pressure = swData->pressure;
		wheel.current_in = swData->current_in;
		msg.wheels.push_back(wheel);
	}
	processDataInputPublisher->publish(msg);
}

void PlatformDriverROS::publishBattery() {
	std_msgs::msg::Float32 msg;
	double volt = 0;
	for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
		double x = driver->getWheelProcessData(i)->voltage_bus;
		if (x > volt)
			volt = x;
	}	
	msg.data = volt;
	batteryPublisher->publish(msg);
}

void PlatformDriverROS::publishIMU() {
	for (unsigned int i=0; i<wheelConfigs.size(); i++) {
		// TODO : need to add wheel number to IMU data to prevent confusion
		// txpdo1_t* swData = driver->getWheelProcessData(i);
		// sensor_msgs::Imu imu;
		// imu.angular_velocity.x = swData->gyro_x;
		// imu.angular_velocity.y = swData->gyro_y;
		// imu.angular_velocity.z = swData->gyro_z;
		// imu.linear_acceleration.x = swData->accel_x;
		// imu.linear_acceleration.y = swData->accel_y;
		// imu.linear_acceleration.z = swData->accel_z;
		// imuPublisher.publish(imu);
	}
}

void PlatformDriverROS::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
	joyCallbackImpl(joy);
}

void PlatformDriverROS::joyCallbackImpl(const sensor_msgs::msg::Joy::SharedPtr joy) {
	if (joy->buttons[5]) {
		useJoy = true;

		if (prev_axes[5] <= 0 && joy->axes[5] > 0.5 && joyScale < 1.0) {
			joyScale = joyScale * 2.0;
			if (joyScale > 1.0)
				joyScale = 1.0;
			std::cout << "New joypad maxvel = " << joyScale * joyVlinMax << " m/s" << std::endl;
		} else if (prev_axes[5] >= 0 && joy->axes[5] < -0.5 && joyScale > 0.001) {
			joyScale = joyScale / 2.0;
			std::cout << "New joypad maxvel = " << joyScale * joyVlinMax << " m/s" << std::endl;
		}

	} else {
		if (useJoy)
			driver->setTargetVelocity(0, 0, 0);

		useJoy = false;
	}

	if (useJoy) {
		driver->setTargetVelocity(joy->axes[1] * joyVlinMax * joyScale, joy->axes[0] * joyVlinMax * joyScale, joy->axes[2] * joyVaMax * joyScale);
		if (activeByJoypad)
			driver->setCanChangeActive();
	}

	if (prev_axes.size() == joy->axes.size()) {
		prev_axes = joy->axes;
	} else {
		std::cout << "Joypad axes dimension does not match. Please check the joypad configuration!" << std::endl;
	}
}

void PlatformDriverROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
	//if (!useJoy && !debugMode)
	if (!useJoy)
		driver->setTargetVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void PlatformDriverROS::currentMaxCallback(const std_msgs::msg::Float32::SharedPtr msg) const {
	if (msg->data >= 0 && msg->data <= currentMax)
		driver->setCurrentDrive(msg->data);
}

void PlatformDriverROS::resetCallback(const std_msgs::msg::Empty::SharedPtr msg) const {
	// only error flags are resetted so far
	RCLCPP_INFO(nh->get_logger(), "Reset error flags.");
	driver->resetErrorFlags();
}

void PlatformDriverROS::enableCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const {
	driver->setWheelsEnable(msg->data);
}


} //namespace kelo
