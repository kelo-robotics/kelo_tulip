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
#include <iostream>

extern "C" {
#include "kelo_tulip/soem/ethercat.h"
#include "kelo_tulip/soem/ethercattype.h"
#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}

namespace kelo {

PlatformDriver::PlatformDriver(const std::vector<WheelConfig>& wheelConfigs, const std::vector<WheelData>& wheelData)
	: wheelConfigs(wheelConfigs)
	, wheelData(wheelData)
{
	if (wheelConfigs.size() != wheelData.size()) { // should not happen
		std::cout << "Error: wheel data structs have inconsistent size\n";
	}

	nWheels = wheelConfigs.size();

	// controller parameters
	currentStop = 1;
	currentDrive = 20;

	maxvlin = 1.5;
	maxva = 1.0;
	maxvlinacc = 0.0025; // per msec, same value for dec
	maxangleacc = 0.01; // at vlin=0, per msec, same value for dec
	maxvaacc = 0.01; // per msec, same value for dec
	
	// status variables
	state = DRIVER_STATE_INIT;
	statusError = false;
	timestampError = false;
	canChangeActive = false;
	showedMessageChangeActive = false;
	stepCount = 0;

	wheelsetpointmin = 0.01;
	wheelsetpointmax = 35.0;

	current_ts = 0;
	swErrorCount = 0;
	ethercatWkcError = false;

	flagReconnectSlave = false;

	encoderInitialized = false;
	sum_encoder.resize(nWheels, std::vector<double> (2, 0));
	prev_encoder.resize(nWheels, std::vector<double> (2, 0));
	wheel_setpoint_ts.resize(nWheels, 0);
	wheel_sensor_ts.resize(nWheels, 0);
	processData.resize(nWheels);
	lastProcessData.resize(nWheels);

	velocityPlatformController.initialise(wheelConfigs);
}

PlatformDriver::~PlatformDriver() {
}

PlatformDriver::PlatformDriver(const PlatformDriver&) {
}

bool PlatformDriver::initEtherCAT2(ecx_contextt* ecx_context, int ecx_slavecount) {
	this->ecx_contextp = ecx_context;
	return true;
}

bool PlatformDriver::initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount) {
	this->ecx_slaves = ecx_slaves;

	std::cout << "PlatformDriver InitEtherCAT\n";
	for (unsigned int i = 0; i < wheelConfigs.size(); i++) {
		int slave = wheelConfigs[i].ethercatNumber;
		std::cout << "Wheel #" << i << " is slave #" << slave << std::endl; 
		if (slave > ecx_slavecount) { // slaves start index 1
			std::cout << "Found only " << ecx_slavecount << " EtherCAT slaves, but config requires at least " << slave << std::endl; 
			return false;
		}

		if (ecx_slaves[slave].eep_id != 24137745 && ecx_slaves[slave].eep_id != 0 && ecx_slaves[slave].eep_id != 0x17010091 && ecx_slaves[slave].eep_id != 0x02001001) {
			std::cout << "EtherCAT slave #" << i << " has wrong id: " << ecx_slaves[slave].eep_id << std::endl;
			return false;
		}
	}

	return true;
}

bool PlatformDriver::step() {
	stepCount++;
	
/*
	for (unsigned int i = 0; i < nWheels; i++) {
		int slave = wheelConfigs[i].ethercatNumber;
		if (ecx_slaves[slave].state == EC_STATE_SAFE_OP + EC_STATE_ERROR) {
			std::cout << "Trying to reconnect slave " << slave << std::endl;
	
		//if (flagReconnectSlave) {	
			ecx_slaves[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
			ecx_writestate(&ecx_context, slave);
		  ecx_statecheck(&ecx_context, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
			ecx_readstate(&ecx_context);
		  if (ecx_slaves[slave].state != EC_STATE_SAFE_OP) {
		    std::cout << "Failed to reset slave to SAFE_OP.\n";
			} else {
				ecx_slaves[slave].state = EC_STATE_OPERATIONAL;
		
			  ecx_send_processdata(&ecx_context);
			  ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
				ecx_writestate(&ecx_context, slave);
			  ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
				ecx_readstate(&ecx_context);
			  if (ecx_slaves[slave].state != EC_STATE_OPERATIONAL) {
			    std::cout << "Failed to reset slave " << slave << " to OP.\n";
				} else {
			   std::cout << "Returned slave " << slave << " to OP state.\n";
			  }
			}
		//flagReconnectSlave = false;
		}
	}
*/

	lastProcessData = processData;
	for (unsigned int i = 0; i < nWheels; i++)
		processData[i] = *getWheelProcessData(i);

	// TODO check if should take timestamp differently, or from each wheel separately
	if (nWheels > 0)
		current_ts = processData[0].sensor_ts; // TODO: atleast use firstWheel

	updateStatusError();
	updateEncoders();

	switch (state) {
		case DRIVER_STATE_INIT:   return stepInit();
		case DRIVER_STATE_READY:  return stepReady();
		case DRIVER_STATE_ACTIVE: return stepActive();
		case DRIVER_STATE_ERROR:  return stepError();
		default: 
			doStop();
	}

	return true;
}

// in state init: wait and don't move until all wheels are enabled
bool PlatformDriver::stepInit() {
	doStop();

	bool ready = true;
	for (unsigned int wheel = 0; wheel < nWheels; wheel++)
		if (!hasWheelStatusEnabled(wheel) || hasWheelStatusError(wheel))
			ready = false;
	
	if (ready) {
		state = DRIVER_STATE_READY;
		std::cout << "PlatformDriver from INIT to READY" << std::endl;
	}
		
	if (stepCount > 500 && !ready) {
		std::cout << "Stopping platform driver, because wheels don't become ready." << std::endl;
		return false;
	}
	
	return true;
}

bool PlatformDriver::stepReady() {
	doStop();

	if (!statusError) {
		if (canChangeActive) {
			state = DRIVER_STATE_ACTIVE;
			std::cout << "PlatformDriver from READY to ACTIVE" << std::endl;
		}
		else if (!showedMessageChangeActive) {
			std::cout << "platform driver is ready, but waiting for signal to become active." << std::endl;
			showedMessageChangeActive = true;
		}		
	}

	return true;
}

bool PlatformDriver::stepActive() {
	doControl();
	return true;
}

bool PlatformDriver::stepError() {
	// could check some condition if error has been resolved and change to ready state
	doStop();
	return true;
}

void PlatformDriver::setTargetVelocity(double vx, double vy, double va) {
	velocityPlatformController.setPlatformTargetVelocity(vx, vy, va);
}

void PlatformDriver::setCurrentStop(double x) {
	currentStop = x;
}

void PlatformDriver::setCurrentDrive(double x) {
	currentDrive = x;
}

double PlatformDriver::getCurrentDrive() {
	return currentDrive;
}

void PlatformDriver::setCanChangeActive() {
	canChangeActive = true;
}


void PlatformDriver::setMaxvlin(double x) {
	maxvlin = x;
}

double PlatformDriver::getMaxvlin() {
	return maxvlin;
}

void PlatformDriver::setMaxva(double x) {
	maxva = x;
}

double PlatformDriver::getMaxva() {
	return maxva;
}

void PlatformDriver::setMaxvlinacc(double x) {
	maxvlinacc = x;
}

void PlatformDriver::setMaxangleacc(double x) {
	maxangleacc = x;
}

void PlatformDriver::setMaxvaacc(double x) {
	maxvaacc = x;
}

void PlatformDriver::reconnectSlave(int slave) {
	flagReconnectSlave = true;
}

txpdo1_t* PlatformDriver::getWheelProcessData(unsigned int wheel) {
	// TODO: thread synchronization
	int slave = wheelConfigs[wheel].ethercatNumber;
	return (txpdo1_t*) ecx_slaves[slave].inputs;
}

void PlatformDriver::setWheelProcessData(unsigned int wheel, rxpdo1_t* data) {
	// TODO: thread synchronization
	int slave = wheelConfigs[wheel].ethercatNumber;
	rxpdo1_t* ecData = (rxpdo1_t*) ecx_slaves[slave].outputs;
	*ecData = *data;
}

std::vector<double> PlatformDriver::getEncoderValue(int idx) {
	if (idx < 0 || idx >= nWheels) {
		std::cout << "Failed to return encoder value. Encoder index does not match" << std::endl;
		return std::vector<double>();
	}
	
	return sum_encoder[idx];
}

const	WheelData* PlatformDriver::getWheelData(unsigned int wheel) {
	return &wheelData[wheel];
}

bool PlatformDriver::hasWheelStatusEnabled(unsigned int wheel) {
	int status1 = processData[wheel].status1;
	return (status1 & STAT1_ENABLED1) > 0 && (status1 & STAT1_ENABLED2) > 0;
}

bool PlatformDriver::hasWheelStatusError(unsigned int wheel) {
	const int STATUS1a = 3;
	const int STATUS1b = 63;
	const int STATUS1disabled = 60;
	const int STATUS2 = 2051;

	int status1 = processData[wheel].status1;
	int status2 = processData[wheel].status2;

	return (status1 != STATUS1a && status1 != STATUS1b && status1 != STATUS1disabled) || (status2 != STATUS2);
}

void PlatformDriver::updateStatusError() {
	for (unsigned int i = 0; i < nWheels; i++) {
		if (hasWheelStatusError(i)) {
			int s1 = processData[i].status1;
			int s2 = processData[i].status2;		
			if (!statusError) {
				std::cout << "Status error: wheel=" << i << ", status1=" << s1 << ", status2=" << s2 << std::endl;
				statusError = true;
			}
		}

		if (!hasWheelStatusEnabled(i) && state != DRIVER_STATE_INIT) {
			int s1 = processData[i].status1;
			int s2 = processData[i].status2;		
			if (!statusError) {
				std::cout << "Wheel got disabled: wheel=" << i << ", status1=" << s1 << ", status2=" << s2 << std::endl;
				statusError = true;
			}
		}
	}
}

int PlatformDriver::getDriverStatus() {
	int status = (int) state;

	int errorTimestampCrit = checkSmartwheelTimestamp();
	if (errorTimestampCrit == 3)
		status = DRIVER_STATE_ERROR;

	if (timestampError)
		status |= DRIVER_ERROR_TIMESTAMP;

	if (ethercatWkcError) {
		//std::cout << "EtherCAT WKC error" << std::endl;
		status |= DRIVER_ERROR_ETHERCAT_WKC;
	}
	
	if (statusError)
		status |= DRIVER_ERROR_STATUS;

	return status;
}

void PlatformDriver::resetErrorFlags() {
	ethercatWkcError = false;
	statusError = false;
	timestampError = false;
}

int PlatformDriver::checkSmartwheelTimestamp() {
	bool swOK = true;
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* swData = getWheelProcessData(i);
		bool error = false;

		if (wheel_sensor_ts[i] < swData->sensor_ts)
			wheel_sensor_ts[i] = swData->sensor_ts;
		else
			error = true;

		if (wheel_setpoint_ts[i] < swData->setpoint_ts)
			wheel_setpoint_ts[i] = swData->setpoint_ts;
		else
			error = true;
		
		if (error != wheelData[i].errorTimestamp) {
			if (error)
				std::cout << "Timestamp error wheel " << i << ".\n";
			else
				std::cout << "Timestamp wheel " << i << " is ok again.\n";
			wheelData[i].errorTimestamp = error;
		}

		swOK = swOK & !error;
	}
	
	int result = 4;
	if (swOK)
		result = 2;
		
	if (!swOK)
		timestampError = true;
	
	return result;		
}

void PlatformDriver::SetState(int wheel, uint16_t state) {
	int slave = wheelConfigs[wheel].ethercatNumber;
	std::cout << "SetState wheel " << wheel << " slave " << slave << " state " << std::hex << state << std::endl;
	ecx_context.slavelist[slave].state = state;
	ecx_writestate(ecx_contextp, slave);
	std::cout << " SetState done." << std::endl;
}

void PlatformDriver::updateEncoders() {
	if(!encoderInitialized) {
		for (int i = 0; i < nWheels; i++) {
			txpdo1_t* wData = getWheelProcessData(i);
			prev_encoder[i][0] = wData->encoder_1;
			prev_encoder[i][1] = wData->encoder_2;
		}
		encoderInitialized = true;
	}
	
	//count accumulative encoder value
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* wData = getWheelProcessData(i);
		double curr_encoder1 = wData->encoder_1;
		double curr_encoder2 = wData->encoder_2;
		if (fabs(curr_encoder1 - prev_encoder[i][0]) > M_PI) {
			if (curr_encoder1 < prev_encoder[i][0])
				sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0] + 2 * M_PI;
			else
				sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0] - 2 * M_PI;
		} else
			sum_encoder[i][0] += curr_encoder1 - prev_encoder[i][0];
			
		if (fabs(curr_encoder2 - prev_encoder[i][1]) > M_PI) {
			if (curr_encoder2 < prev_encoder[i][1])
				sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1] + 2 * M_PI;
			else
				sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1] - 2 * M_PI;
		} else
			sum_encoder[i][1] += curr_encoder2 - prev_encoder[i][1];	
		
		prev_encoder[i][0] = curr_encoder1;
		prev_encoder[i][1] = curr_encoder2;
	}
}

void PlatformDriver::doStop() {
	rxpdo1_t rxdata;
	rxdata.timestamp = current_ts + 100 * 1000; // TODO
	rxdata.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
	rxdata.limit1_p = currentStop;
	rxdata.limit1_n = -currentStop;
	rxdata.limit2_p = currentStop;
	rxdata.limit2_n = -currentStop;
	rxdata.setpoint1 = 0;
	rxdata.setpoint2 = 0;
	double totalDiffAngle = 0;

	for (unsigned int i = 0; i < nWheels; i++)
		setWheelProcessData(i, &rxdata);
}

void PlatformDriver::doControl() {
	/* initialise struct to be sent to wheels */
	rxpdo1_t rxdata;
	rxdata.timestamp = current_ts + 100 * 1000; // TODO
	rxdata.command1 = COM1_ENABLE1 | COM1_ENABLE2 | COM1_MODE_VELOCITY;
	rxdata.limit1_p = currentDrive;
	rxdata.limit1_n = -currentDrive;
	rxdata.limit2_p = currentDrive;
	rxdata.limit2_n = -currentDrive;
	rxdata.setpoint1 = 0;
	rxdata.setpoint2 = 0;

	// update desired velocity of platform, based on target velocity and veloity ramps
	velocityPlatformController.calculatePlatformRampedVelocities();
	
	for (size_t i = 0; i < nWheels; i++) {
		txpdo1_t* wheel_data = getWheelProcessData(i);

		float setpoint1, setpoint2;

		/* calculate wheel target velocity */
		velocityPlatformController.calculateWheelTargetVelocity(i, wheel_data->encoder_pivot,
                                                                setpoint2, setpoint1);
		setpoint1 *= -1; // because of inverted frame

		/* avoid sending close to zero values */
		if ( fabs(setpoint1) < wheelsetpointmin )
		{
			setpoint1 = 0;
		}
		if ( fabs(setpoint2) < wheelsetpointmin )
		{
			setpoint2 = 0;
		}

		/* avoid sending very large values */
		setpoint1 = Utils::clip(setpoint1, wheelsetpointmax, -wheelsetpointmax);
		setpoint2 = Utils::clip(setpoint2, wheelsetpointmax, -wheelsetpointmax);

		/* send calculated target velocity values to EtherCAT */
		rxdata.setpoint1 = setpoint1;
		rxdata.setpoint2 = setpoint2;
		
		setWheelProcessData(i, &rxdata);
	}
}

} //namespace kelo
