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

#include <iostream>
#include "kelo_tulip/modules/RobileMasterBattery.h"

namespace kelo {

RobileMasterBattery::RobileMasterBattery(int slaveNumber) : EtherCATModule() {
	this->slaveNumber = slaveNumber;

	ecx_slaves = 0;
	
	flagShutdown = false;
	robileCharge = false;
	robileEnablePump = false;
	robileEnableDock = false;
	robileEnableUndock = false;
}

RobileMasterBattery::~RobileMasterBattery() {
}

bool RobileMasterBattery::initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount)  {
	this->ecx_slaves = ecx_slaves;

	if (!(slaveNumber > 0))
		return false;
		
	std::cout << "Robile Master Battery is slave #" << slaveNumber << std::endl; 
	if (slaveNumber > ecx_slavecount) { // slaves start index 1
		std::cout << "Found only " << ecx_slavecount << " EtherCAT slaves, but config requires at least " << slaveNumber << std::endl; 
		return false;
	}
	if (ecx_slaves[slaveNumber].eep_id != 34603264 && ecx_slaves[slaveNumber].eep_id != 0) {
		std::cout << "EtherCAT slave #" << slaveNumber << " has wrong id: " << ecx_slaves[slaveNumber].eep_id << std::endl;
		return false;
	}

	return true;
}

bool RobileMasterBattery::step() {
	if (slaveNumber == 0)
		return false;

 	RobileMasterBatteryProcessDataInput* input = (RobileMasterBatteryProcessDataInput*) ecx_slaves[slaveNumber].inputs;

	// debug output
	//static int istep = 0;
	//if (istep++ > 1000) {
	//	istep = 0;
	//	std::cout << "master battery: status=" << input->Status << ", ts=" << input->TimeStamp << ", Error=" << input->Error<< ", Warning=" << input->Warning << std::endl;
	//}

	RobileMasterBatteryProcessDataOutput data;

	data.Command1 = 0;
	data.Command2 = 0;
	data.Shutdown = 0;
	
	if (flagShutdown)
		data.Shutdown = 0x80;

	boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	if (robileCharge) {
		data.Command2 = 0x01;
	} else if ( robileEnablePump ) {
		double pumpOnDuration = (now - pumpStartTime).total_milliseconds();
		if ( pumpOnDuration > 1000.0 * 5 ) { // 30 seconds; 10 for new mechanism; 5s after ball bearing broke
			robileEnablePump = false;
			std::cout << std::endl << "Disabling pump" << std::endl << std::endl;
			data.Command1 = 0x0;
		}
		data.Command1 = 0x08;
	}	else if ( robileEnableDock ) {
		double dockDuration = (now - dockStartTime).total_milliseconds();
		if ( dockDuration > 1000.0 * 0.5 ) { // 0.5 seconds
			robileEnableDock = false;
			std::cout << std::endl << "Disabling dock" << std::endl << std::endl;
			data.Command1 = 0x0;
		}
		data.Command1 = 0x10;
	} else if ( robileEnableUndock ) {
		double undockDuration = (now - undockStartTime).total_milliseconds();
		if ( undockDuration > 1000.0 * 0.5 ) { // 0.5 seconds
			robileEnableUndock = false;
			std::cout << std::endl << "Disabling undock" << std::endl << std::endl;
			data.Command1 = 0x0;
		}
		data.Command1 = 0x20;
	}	else {
		data.Command1 = 0x0;
	}

	*((RobileMasterBatteryProcessDataOutput*) ecx_slaves[slaveNumber].outputs) = data;		

	return true;
}

void RobileMasterBattery::shutdown(int seconds) {
	// ignore seconds for now
	flagShutdown = true;
}

void RobileMasterBattery::pump(bool enablePump) {
	std::cout << "inside SmartWheelDriver::robileMasterBatteryPump " << enablePump << std::endl;
	robileEnablePump = enablePump;
	if (enablePump) {
		pumpStartTime = boost::posix_time::microsec_clock::local_time();
		std::cout << std::endl << "Enabling pump" << std::endl << std::endl;
	}
}

void RobileMasterBattery::dock(bool dock) {
	std::cout << "inside SmartWheelDriver::robileMasterBatteryDock " << dock << std::endl;
	robileEnableDock = dock;
	if ( dock ) {
		dockStartTime = boost::posix_time::microsec_clock::local_time();
		std::cout << std::endl << "Enabling dock" << std::endl << std::endl;
	}
}

void RobileMasterBattery::undock(bool undock) {
	std::cout << "inside SmartWheelDriver::robileMasterBatteryUndock " << undock << std::endl;
	robileEnableUndock = undock;
	if ( undock ) {
		undockStartTime = boost::posix_time::microsec_clock::local_time();
		std::cout << std::endl << "Enabling undock" << std::endl << std::endl;
	}
}

void RobileMasterBattery::startCharge() {
	robileCharge = true;
}

void RobileMasterBattery::stopCharge() {
	robileCharge = false;
}



} //namespace kelo
