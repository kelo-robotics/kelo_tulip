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

#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

namespace kelo {

RobileMasterBatteryROS::RobileMasterBatteryROS() : EtherCATModuleROS()
{
	battery = NULL;
	topicPrefix = "robile_master_battery/";
}

RobileMasterBatteryROS::~RobileMasterBatteryROS() {
}

bool RobileMasterBatteryROS::init(ros::NodeHandle& nh, std::string configPrefix) {
	// get EtherCAT slave number
	int ethercatNumber = 0;
	nh.param(configPrefix + "ethercat_number", ethercatNumber, 0);
	if (!ethercatNumber > 0) {
		// Try deprecated alternative method
		nh.param("robile_master_battery_ethercat_number", ethercatNumber, 0);
		if (!ethercatNumber > 0) {
			std::cerr << "EtherCAT number for robile master battery not set." << std::endl;
			return false;
		}
	}

	// create EtherCAT module
	battery = new RobileMasterBattery(ethercatNumber);
	if (!battery)
		return false;

	// setup publishers and subscribers
	batteryPublisher = nh.advertise<std_msgs::Float32>(topicPrefix + "voltage", 10);
	processDataInputPublisher = nh.advertise<std_msgs::Float64MultiArray>(topicPrefix + "ethercat_input", 10);

	resetErrorSubscriber = nh.subscribe(topicPrefix + "reset_error", 1, &RobileMasterBatteryROS::callbackResetError, this);
	shutdownSubscriber = nh.subscribe(topicPrefix + "shutdown", 1, &RobileMasterBatteryROS::callbackShutdown, this);
	chargerStartSubscriber = nh.subscribe("charger_start", 1, &RobileMasterBatteryROS::callbackChargerStart, this);
	chargerStopSubscriber = nh.subscribe("charger_stop", 1, &RobileMasterBatteryROS::callbackChargerStop, this);

	return true;
}

bool RobileMasterBatteryROS::step() {
	// publish voltage
	std_msgs::Float32 msgBattery;
	msgBattery.data = battery->getVoltage();
	batteryPublisher.publish(msgBattery);

	publishEthercatInput();

	return true;
}

std::string RobileMasterBatteryROS::getType() {
	return "robile_master_battery";
};

EtherCATModule* RobileMasterBatteryROS::getEtherCATModule() {
	return battery;
}

void RobileMasterBatteryROS::callbackResetError(const std_msgs::Empty& msg) {
	battery->resetError();
}

void RobileMasterBatteryROS::callbackShutdown(const std_msgs::Int32& msg) {
	battery->shutdown(msg.data);
}

void RobileMasterBatteryROS::callbackChargerStart(const std_msgs::Int32& msg) {
	battery->startCharge();
}

void RobileMasterBatteryROS::callbackChargerStop(const std_msgs::Int32& msg) {
	battery->stopCharge();
}

void RobileMasterBatteryROS::publishEthercatInput() {
	std_msgs::Float64MultiArray msg;
	const RobileMasterBatteryProcessDataInput* input = battery->getProcessDataInput();
	std_msgs::MultiArrayDimension dim;
	dim.size = 1;

	dim.label = "timestamp";
	msg.data.push_back(input->TimeStamp);
	msg.layout.dim.push_back(dim);

	dim.label = "status"; 
	dim.stride++;
	msg.data.push_back(input->Status);
	msg.layout.dim.push_back(dim);

	dim.label = "error";
	dim.stride++;
	msg.data.push_back(input->Error);
	msg.layout.dim.push_back(dim);

	dim.label = "warning";
	dim.stride++;
	msg.data.push_back(input->Warning);
	msg.layout.dim.push_back(dim);

	dim.label = "output_current";
	dim.stride++;
	msg.data.push_back(input->OutputCurrent);
	msg.layout.dim.push_back(dim);

	dim.label = "output_voltage";
	dim.stride++;
	msg.data.push_back(input->OutputVoltage);
	msg.layout.dim.push_back(dim);

	dim.label = "output_power";
	dim.stride++;
	msg.data.push_back(input->OutputPower);
	msg.layout.dim.push_back(dim);

	dim.label = "aux_port_current";
	dim.stride++;
	msg.data.push_back(input->AuxPortCurrent);
	msg.layout.dim.push_back(dim);

	dim.label = "generic_data1";
	dim.stride++;
	msg.data.push_back(input->GenericData1);
	msg.layout.dim.push_back(dim);

	dim.label = "generic_data2";
	dim.stride++;
	msg.data.push_back(input->GenericData2);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_PwrDeviceId";
	dim.stride++;
	msg.data.push_back(input->bmsm_PwrDeviceId);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_Status";
	dim.stride++;
	msg.data.push_back(input->bmsm_Status);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_Voltage";
	dim.stride++;
	msg.data.push_back(input->bmsm_Voltage);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_Current";
	dim.stride++;
	msg.data.push_back(input->bmsm_Current);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_Temperature";
	dim.stride++;
	msg.data.push_back(input->bmsm_Temperature);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_SOC";
	dim.stride++;
	msg.data.push_back(input->bmsm_SOC);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_SN";
	dim.stride++;
	msg.data.push_back(input->bmsm_SN);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_BatData1";
	dim.stride++;
	msg.data.push_back(input->bmsm_BatData1);
	msg.layout.dim.push_back(dim);

	dim.label = "bmsm_BatData2";
	dim.stride++;
	msg.data.push_back(input->bmsm_BatData2);
	msg.layout.dim.push_back(dim);

	processDataInputPublisher.publish(msg);
}

} //namespace kelo
