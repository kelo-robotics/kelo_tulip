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

#include "kelo_tulip/EtherCATMaster.h"
#include "kelo_tulip/PlatformDriverROS.h"
#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include "rclcpp/rclcpp.hpp"

// create and configure one module
kelo::EtherCATModuleROS* createModule(rclcpp::Node::SharedPtr nh, std::string moduleType, std::string moduleName, std::string configTag) {
	kelo::EtherCATModuleROS* module = NULL;	
	if (moduleType == "robile_master_battery") {
		module = new kelo::RobileMasterBatteryROS();
	} else if (moduleType == "platform_driver") {
		module = new kelo::PlatformDriverROS();
	} else {
		std::cout << "Unknown module type: " << moduleType << std::endl;
		return NULL;
	}

	if (!module) {
		std::cout << "Module " << moduleName << " could not be created." << std::endl;
		return NULL;
	}

	// Initialize module and if not possible, delete it.
	if (!module->init(nh, configTag)) {
		std::cout << "Failed to initialize module " << moduleName << "." << std::endl;
		delete module;
		return NULL;
	}
	
	return module;
}

/*
// step through all modules
void stepModules(const rclcpp::TimerEvent&) {
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
}
*/

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("platform_driver");

	nh->declare_parameter("modules.list", std::vector<std::string>{}); 
	nh->declare_parameter("start_retry_delay", 0);
	nh->declare_parameter("robile_master_battery_ethercat_number", 0);
	nh->declare_parameter("device", "");

	std::vector<kelo::EtherCATModuleROS*> rosModules;

	// create modules by iterating through struct in config
	std::string configModulesTag = "modules";
	std::vector<std::string> moduleList = nh->get_parameter(configModulesTag + ".list").as_string_array();
	for (unsigned int i = 0; i < moduleList.size(); i++) {
		nh->declare_parameter(configModulesTag + "." + moduleList[i] + ".type", "");
		std::string moduleName = moduleList[i];
		std::string moduleType = nh->get_parameter(configModulesTag + "." + moduleList[i] + ".type").as_string();
		std::string configTag = configModulesTag + "." + moduleName + ".";
		kelo::EtherCATModuleROS* module = createModule(nh, moduleType, moduleName, configTag);
		
		if (!module)
			return -1;
			
		rosModules.push_back(module);
	}

	// legacy config mode for master battery
	int robileMasterBatteryEthercatNumber = nh->get_parameter("robile_master_battery_ethercat_number").as_int();
	if (robileMasterBatteryEthercatNumber > 0) {	
		kelo::EtherCATModuleROS* module = new kelo::RobileMasterBatteryROS();
		if (!module || !module->init(nh, ""))
			return -1;
			
		rosModules.push_back(module);		
	}

	// collect EtherCAT modules
	std::vector<kelo::EtherCATModule*> etherCATmodules;
	for (size_t i = 0; i < rosModules.size(); i++)
		etherCATmodules.push_back(rosModules[i]->getEtherCATModule());

	// create and configure EtherCAT master
	std::string device = nh->get_parameter("device").as_string();
	int delayRetry = nh->get_parameter("start_retry_delay").as_int();

	kelo::EtherCATMaster* master = new kelo::EtherCATMaster(device, etherCATmodules);
	if (!master) {
		std::cout << "Failed to create EtherCAT master." << std::endl;
		return -1;		
	}

	// initialize EtherCAT
	while (!master->initEthercat()) {
		if (delayRetry == 0) {
			RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT");
			return -1;
		}
		RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT, will retry in %d s.", delayRetry);
		std::this_thread::sleep_for(std::chrono::seconds(delayRetry));
	}
	
	// ROS main loop
	rclcpp::Rate rate(20.0f); // hz
	while (rclcpp::ok()) {
		rclcpp::spin_some(nh);		

		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
			
		rate.sleep();
	}

	// delete and close everything
	for (size_t i = 0; i < rosModules.size(); i++)
		delete rosModules[i];

	delete master;
	
	rclcpp::shutdown();
	return 0;
}

