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
#include <ros/ros.h>

// create and configure one module
kelo::EtherCATModuleROS* createModule(ros::NodeHandle& nh, std::string moduleType, std::string moduleName, std::string configTag) {
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
void stepModules(const ros::TimerEvent&) {
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
}
*/

int main (int argc, char** argv)
{
	ros::init (argc, argv, "platform_driver");
	ros::NodeHandle nh("~");

	std::vector<kelo::EtherCATModuleROS*> rosModules;
	
	// create modules by iterating through struct in config
	XmlRpc::XmlRpcValue modulesXML;
	std::string configModulesTag = "modules";
	nh.getParam(configModulesTag, modulesXML);
	if (modulesXML.getType() == XmlRpc::XmlRpcValue::TypeStruct)	{
		for (XmlRpc::XmlRpcValue::const_iterator it = modulesXML.begin(); it != modulesXML.end(); ++it) {
			if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)	{
				std::cout << "Error: configuration for module " << it->first << " cannot be read." << std::endl;
				return -1;
			}

			std::string moduleName = it->first;
			std::string moduleType = it->second["type"];
			std::string configTag = configModulesTag + "/" + moduleName + "/";
			kelo::EtherCATModuleROS* module = createModule(nh, moduleType, moduleName, configTag);
			
			if (!module)
				return -1;
				
			rosModules.push_back(module);
		}
	}

	// legacy config mode for master battery
	int robileMasterBatteryEthercatNumber = 0;
	nh.param("robile_master_battery_ethercat_number", robileMasterBatteryEthercatNumber, 0);
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
	std::string device;
	nh.getParam("device", device);

	double delayRetry = 0;
	nh.getParam("start_retry_delay", delayRetry);

	kelo::EtherCATMaster* master = new kelo::EtherCATMaster(device, etherCATmodules);
	if (!master) {
		std::cout << "Failed to create EtherCAT master." << std::endl;
		return -1;		
	}

	// initialize EtherCAT
	while (!master->initEthercat()) {
		if (delayRetry == 0) {
			ROS_ERROR("Failed to initialize EtherCAT");
			return -1;
		}
		ROS_ERROR("Failed to initialize EtherCAT, will retry in %.2f s.", delayRetry);
		ros::WallDuration(delayRetry).sleep();
	}
	
	// ROS main loop
	ros::Rate rate(20.0f); // hz
	while (ros::ok()) {
		ros::spinOnce();		

		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
			
		rate.sleep();
	}
	

	// delete and close everything
	for (size_t i = 0; i < rosModules.size(); i++)
		delete rosModules[i];

	delete master;
	
	ros::shutdown();
	return 0;
}

