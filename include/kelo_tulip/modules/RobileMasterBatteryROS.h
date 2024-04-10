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


#ifndef MODULES_ROBILEMASTERBATTERYROS_H
#define MODULES_ROBILEMASTERBATTERYROS_H

#include <memory>

#include "kelo_tulip/modules/RobileMasterBattery.h"
#include "kelo_tulip/EtherCATModuleROS.h"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using std::placeholders::_1;

namespace kelo {

//! ROS interface for RobileMasterBattery
//! Currently only provides two topics, for shutdown and to get battery voltage.

class RobileMasterBatteryROS : public EtherCATModuleROS {
public:
	RobileMasterBatteryROS();
	virtual ~RobileMasterBatteryROS();

	virtual bool init(rclcpp::Node::SharedPtr nh, std::string configPrefix);

	virtual bool step();

	virtual std::string getType();

	virtual EtherCATModule* getEtherCATModule();
	
protected:
	void callbackResetError(const std_msgs::msg::Empty::SharedPtr msg) const;
	void callbackShutdown(const std_msgs::msg::Int32::SharedPtr msg) const;
	void callbackChargerStart(const std_msgs::msg::Int32::SharedPtr msg) const;
	void callbackChargerStop(const std_msgs::msg::Int32::SharedPtr msg) const;

	void publishEthercatInput();

	RobileMasterBattery* battery;
	
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batteryPublisher;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr processDataInputPublisher;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetErrorSubscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr shutdownSubscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chargerStartSubscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chargerStopSubscriber;

	std::string topicPrefix;
};

} // namespace kelp

#endif // MODULES_ROBILEMASTERBATTERYROS_H
