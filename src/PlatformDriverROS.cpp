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


#include "kelo_tulip/PlatformDriver.h"
#include "kelo_tulip/modules/RobileMasterBattery.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/String.h>
//#include <std_msgs/UInt64MultiArray.h>
//#include <std_msgs/UInt8.h>
//#include <std_msgs/UInt8MultiArray.h>
//#include <tf/transform_broadcaster.h>

kelo::PlatformDriver* driver;
std::vector<kelo::WheelConfig> wheelConfigs;
std::vector<kelo::WheelData> wheelData;
kelo::RobileMasterBattery* robileMasterBattery = 0;

ros::Publisher odomPublisher;
ros::Publisher odomInitializedPublisher;
ros::Publisher mileagePublisher;
ros::Publisher imuPublisher;
//ros::Publisher valuesPublisher;
ros::Publisher batteryPublisher;
ros::Publisher errorPublisher;
//ros::Publisher timestampPublisher;
ros::Publisher statusPublisher;

nav_msgs::Odometry odom;
tf::TransformBroadcaster* odom_broadcaster;


//PARAMETERS
double s_w = 0.01; //caster offset of a smartWheel
double d_w = 0.0775; //distance between the left and the right wheel
double s_d_ratio = s_w / d_w;	
//double r_w = 0.074; //the radius of the wheel
double r_w = 0.0524; //the radius of the wheel

int nWheels = 0;

bool useJoy = false;
bool debugMode = false;
bool activeByJoypad = false;

double currentMax = 20;

double joyVlinMax = 1.0;
double joyVaMax = 1.0;
double joyScale = 1.0;

std::vector<double> prev_left_enc;
std::vector<double> prev_right_enc;
double odomx = 0;
double odomy = 0;
double odoma = 0;

void readWheelConfig(const ros::NodeHandle& nh) {
	for (int i = 0; i < nWheels; i++) {
		std::stringstream ssGroupName;
		ssGroupName << "wheel" << i;
		std::string groupName = ssGroupName.str();

		kelo::WheelConfig config;
		config.enable = true;
		config.reverseVelocity = true;
		bool ok =		
		     nh.getParam(groupName + "/ethercat_number", config.ethercatNumber)
		  && nh.getParam(groupName + "/x", config.x)
			&& nh.getParam(groupName + "/y", config.y)
			&& nh.getParam(groupName + "/a", config.a);

		int critical = 1;
		if (nh.getParam(groupName + "/critical", critical))
			config.critical = critical;

		int reverseVelocity = 0;
		if (nh.getParam(groupName + "/reverse_velocity", reverseVelocity))
			config.reverseVelocity = (reverseVelocity != 0);

		if (!ok)
			ROS_WARN("Missing config value for wheel %d", i);

		wheelConfigs[i] = config;
	}
}

void checkAndPublishSmartWheelStatus() {
	int state = 0;
	int errors = 0;
//	for (int i = 0; i < nMasters; i++) { // TODO check
		int mstatus = driver->getDriverStatus();
		int mstate = (mstatus & 0x000000ff);
		int merror = (mstatus & 0xffffff00);
		
		if (mstate > state)
			state = mstate;
			
		errors |= merror;	
//	}
	int status = state | errors;
	
	std_msgs::Int32 statusMsg;
	statusMsg.data = status;
	statusPublisher.publish(statusMsg);

	std_msgs::Int32 errorMsg;
	if (errors) {
		// TODO correct
		//stop navigation and start debug mode. Robot can only be moved with joystick
		debugMode = true;
		errorMsg.data = status;
		errorPublisher.publish(errorMsg);
		statusPublisher.publish(statusMsg);
	} else {
		if (debugMode) {
			debugMode = false;
			errorMsg.data = 0;
			errorPublisher.publish(errorMsg);
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

void initializeEncoderValue() {
	prev_left_enc.resize(nWheels, 0);
	prev_right_enc.resize(nWheels, 0);
	for (int i=0; i<nWheels; i++) {
		txpdo1_t* swDataInit = driver->getProcessData(wheelConfigs[i].ethercatNumber);
		std::vector<double> encoderValueInit = driver->getEncoderValue(i);
		prev_left_enc[i] = encoderValueInit[0];
		prev_right_enc[i] = encoderValueInit[1];
	}
}

void calculateRobotVelocity(double& vx, double& vy, double& va, double& encDisplacement) {
	double dt = 0.05;
	
	//initialize the variables
	vx = 0;
	vy = 0;
	va = 0;
	encDisplacement = 0;
	
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = driver->getProcessData(wheelConfigs[i].ethercatNumber);
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

void calculateRobotPose(double vx, double vy, double va) {
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

void publishOdometry(double vx, double vy, double va) {
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odoma);
	odom.header.stamp = ros::Time::now();
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
	odom.pose.pose.orientation = odom_quat;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = va;
	odomPublisher.publish(odom);
}
		
void createOdomToBaseLinkTransform(geometry_msgs::TransformStamped& odom_trans) {
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odoma);
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = odomx;
	odom_trans.transform.translation.y = odomy;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
}

void publishBattery() {
	std_msgs::Float32 msg;
	double volt = 0;
	for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
		double x = driver->getProcessData(wheelConfigs[i].ethercatNumber)->voltage_bus;
		if (x > volt)
			volt = x;
	}	
	msg.data = volt;
	batteryPublisher.publish(msg);
}

void publishIMU () {
	for (unsigned int i=0; i<wheelConfigs.size(); i++) {
		txpdo1_t* swData = driver->getProcessData(wheelConfigs[i].ethercatNumber);
		sensor_msgs::Imu imu;
		imu.angular_velocity.x = swData->gyro_x;
		imu.angular_velocity.y = swData->gyro_y;
		imu.angular_velocity.z = swData->gyro_z;
		imu.linear_acceleration.x = swData->accel_x;
		imu.linear_acceleration.y = swData->accel_y;
		imu.linear_acceleration.z = swData->accel_z;
		imuPublisher.publish(imu);
	}
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	if (joy->buttons[5]) {
		useJoy = true;

		if (joy->axes[5] > 0.5 && joyScale < 1.0) {
			joyScale = joyScale * 2.0;
			if (joyScale > 1.0)
				joyScale = 1.0;
			std::cout << "New joypad maxvel = " << joyScale * joyVlinMax << " m/s" << std::endl;
		} else if (joy->axes[5] < -0.5 && joyScale > 0.001) {
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
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	//if (!useJoy && !debugMode)
	if (!useJoy)
		driver->setTargetVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void currentMaxCallback(const std_msgs::Float32& msg) {
	if (msg.data >= 0 && msg.data <= currentMax)
		driver->setCurrentDrive(msg.data);
}

void resetCallback(const std_msgs::Empty& msg) {
	// only error flags are resetted so far
	ROS_INFO("Reset error flags.");
	driver->resetErrorFlags();
}

void publishAll(const ros::TimerEvent&) {
		//check condition of the smartwheels and handle the error
		checkAndPublishSmartWheelStatus();

		//calculate robot velocity
		double vx, vy, va, encDisplacement;
		calculateRobotVelocity(vx, vy, va, encDisplacement);

		//calculate robot displacement and current pose
		calculateRobotPose(vx, vy, va);
			
		//publish the odometry
		publishOdometry(vx, vy, va);

		//broadcast odom-base_link transform
		geometry_msgs::TransformStamped odom_trans;
		createOdomToBaseLinkTransform(odom_trans);
		odom_broadcaster->sendTransform(odom_trans);
		
/*
		//publish smartwheel values
		std_msgs::Float64MultiArray processDataValues;
		for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
			addToWheelDataMsg(processDataValues, driver->getWheelData(i));
			addToProcessDataMsg(processDataValues, driver->getProcessData(wheelConfigs[i].ethercatNumber));
			processDataValues.data.push_back(driver->getCurrentDrive());
			processDataValues.data.push_back(driver->getThreadPhase());
		}
		valuesPublisher.publish(processDataValues);
*/

		publishBattery();
		
		//publish IMU data
		publishIMU();
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "platform_driver");
	ros::NodeHandle nh("~");
	
	nh.getParam("num_wheels", nWheels);
	
	if (nWheels == 0) {
		ROS_ERROR("Missing number of wheels in config file");
		return -1;
	}

	wheelConfigs.resize(nWheels);
	kelo::WheelData data = {};
	data.enable = true;
	data.error = false;
	data.errorTimestamp = false;
	wheelData.resize(nWheels, data);

	// read all wheel configs
	readWheelConfig(nh);

	// read config and create driver
	int wheelIndex = 0;	

	std::vector<kelo::EtherCATModule*> modules;

	int robileMasterBatteryEthercatNumber = 0;
	nh.param("robile_master_battery_ethercat_number", robileMasterBatteryEthercatNumber, 0);
	if (robileMasterBatteryEthercatNumber > 0) {
		robileMasterBattery = new kelo::RobileMasterBattery(robileMasterBatteryEthercatNumber);
		modules.push_back(robileMasterBattery);
	}

	std::string device;
	int nWheelsMaster;
	bool ok = nh.getParam("device", device);

//TODO check
		
	bool hasNumWheel = nh.getParam("num_wheels", nWheelsMaster);
    int firstWheel = 0;

	driver = new kelo::PlatformDriver(device, modules, &wheelConfigs, &wheelData, firstWheel, nWheelsMaster);

	wheelIndex += nWheelsMaster;

	// set driver control parameters		
	double x;
	if (nh.getParam("wheel_distance", x))
		driver->setWheelDistance(x);
	if (nh.getParam("wheel_diameter", x))
		driver->setWheelDiameter(x);
	if (nh.getParam("current_stop", x))
		driver->setCurrentStop(x);
	if (nh.getParam("current_drive", x))
		driver->setCurrentDrive(x);

	if (nh.getParam("vlin_max", x))
		driver->setMaxvlin(x);
	if (nh.getParam("va_max", x))
		driver->setMaxva(x);
	if (nh.getParam("vlin_acc_max", x))
		driver->setMaxvlinacc(x);
	if (nh.getParam("angle_acc_max", x))
		driver->setMaxangleacc(x);
	if (nh.getParam("va_acc_max", x))
		driver->setMaxvaacc(x);

	joyVlinMax = driver->getMaxvlin();
	joyVaMax = driver->getMaxva();
	if (nh.getParam("joy_vlin_max", x))
		joyVlinMax = x;
	if (nh.getParam("joy_va_max", x))
		joyVaMax = x;
	if (nh.getParam("joy_scale", x))
		if (x > 0 && x <= 1.0)
			joyScale = x;

	bool b;
	if (nh.getParam("active_by_joypad", b))
		activeByJoypad = b;
	if (!activeByJoypad)
		driver->setCanChangeActive();
		
	double delay = 0;
	if (nh.getParam("start_delay", delay)) {
		ROS_INFO("smart_wheel_driver start_delay = %.2f s, waiting.", delay);
		ros::WallDuration(delay).sleep();
	}

	if (nh.getParam("current_max", x)) {
		currentMax = x;
	}

	double delayRetry = 0;
	nh.getParam("start_retry_delay", delayRetry);
		
	// initialize Ethercat
	while (!driver->initEthercat()) {
		if (delayRetry == 0) {
			ROS_ERROR("Failed to initialize EtherCAT");
			return -1;
		}
		ROS_ERROR("Failed to initialize EtherCAT, will retry in %.2f s.", delayRetry);
		ros::WallDuration(delayRetry).sleep();
	}
	

	ros::NodeHandle nhGlobal("");
	odomPublisher = nhGlobal.advertise<nav_msgs::Odometry>("odom", 10);
	odomInitializedPublisher = nhGlobal.advertise<std_msgs::Empty>("odom_initialized", 10);
//	timestampPublisher = nh.advertise<std_msgs::UInt64MultiArray>("timestamp", 10);
	imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 10);
	batteryPublisher = nh.advertise<std_msgs::Float32>("battery", 10);
//	errorPublisher = nh.advertise<std_msgs::Int32>("error", 10);
	statusPublisher = nh.advertise<std_msgs::Int32>("status", 10);
	ros::Subscriber joySubscriber = nh.subscribe("/joy", 1000, joyCallback);
	ros::Subscriber cmdVelSubscriber = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);
	ros::Subscriber resetSubscriber = nh.subscribe("reset", 1, resetCallback);
//	ros::Subscriber currentMaxSubscriber = nh.subscribe("current_max", 1, currentMaxCallback);
	odom_broadcaster = new tf::TransformBroadcaster();

	initializeEncoderValue();

	ros::Timer timer = nh.createTimer(ros::Duration(0.05), publishAll);
	ros::spin();

	ros::shutdown();
	return 0;
}

