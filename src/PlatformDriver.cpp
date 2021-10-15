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

PlatformDriver::PlatformDriver(std::string device, std::vector<EtherCATModule*> modules,
			std::vector<WheelConfig>* wheelConfigs, std::vector<WheelData>* wheelData, int firstWheel, int nWheels)
	: modules(modules)
	, wheelConfigs(wheelConfigs)
	, wheelData(wheelData)
    , firstWheel(firstWheel)
	, nWheels(nWheels)
{
	this->device = device;
	
	if (wheelConfigs->size() != wheelData->size() || nWheels > wheelConfigs->size()) { // should not happen
		std::cout << "SmartWheel error: wheel data structs have inconsistent size\n";
	}

	// controller parameters
	wheelDistance = 0.0775;
	wheelDiameter = 0.104;
	
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
	stallError = false;
	slipError = false;
	timestampError = false;
	canChangeActive = false;
	showedMessageChangeActive = false;

	wheelsetpointmin = 0.01;
	wheelsetpointmax = 35.0;

	EcatError = FALSE;
	current_ts = 0;
	swErrorCount = 0;
	ethercatWkcError = false;

	ethercatInitialized = false;
	threadPhase = 0;
	pauseThreadMs = 0;
	flagReconnectSlave = false;

	encoderInitialized = false;
	sum_encoder.resize(nWheels, std::vector<double> (2, 0));
	prev_encoder.resize(nWheels, std::vector<double> (2, 0));
	wheel_setpoint_ts.resize(nWheels, 0);
	wheel_sensor_ts.resize(nWheels, 0);
	processData.resize(nWheels);
	lastProcessData.resize(nWheels);

	ecx_context.port = &ecx_port;
	ecx_context.slavelist = &ecx_slave[0];
	ecx_context.slavecount = &ecx_slavecount;
	ecx_context.maxslave = EC_MAXSLAVE;
	ecx_context.grouplist = &ec_group[0];
	ecx_context.maxgroup = EC_MAXGROUP;
	ecx_context.esibuf = &esibuf[0];
	ecx_context.esimap = &esimap[0];
	ecx_context.esislave = 0;
	ecx_context.elist = &ec_elist;
	ecx_context.idxstack = &ec_idxstack;
	ecx_context.ecaterror = &EcatError;
	ecx_context.DCtO = 0;
	ecx_context.DCl = 0;
	ecx_context.DCtime = &ec_DCtime;
	ecx_context.SMcommtype = &ec_SMcommtype;
	ecx_context.PDOassign = &ec_PDOassign;
	ecx_context.PDOdesc = &ec_PDOdesc;
	ecx_context.eepSM = &ec_SM;
	ecx_context.eepFMMU = &ec_FMMU;

	velocityPlatformController.initialise(*wheelConfigs);
}

PlatformDriver::~PlatformDriver() {
}

PlatformDriver::PlatformDriver(const PlatformDriver&) {
}

void PlatformDriver::setTargetVelocity(double vx, double vy, double va) {

    velocityPlatformController.setPlatformTargetVelocity(vx, vy, va);
}

void PlatformDriver::setWheelDistance(double x) {
	wheelDistance = x;
}

void PlatformDriver::setWheelDiameter(double x) {
	wheelDiameter = x;
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

bool PlatformDriver::initEthercat() {
	if (!ethercatInitialized) {
		if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) {
			std::cout << "Failed to initialize EtherCAT on " << device << " with communication thread\n";
			return false;
		}
		std::cout << "Initializing EtherCAT on " << device << "\n";
		
		ethercatInitialized = true;
	}

	// find and auto-config slaves
	int wkc = ecx_config_init(&ecx_context, TRUE);
	if (!wkc) {
		std::cout << "No EtherCAT slaves were found.\n";
		return false;
	}

	ecx_config_map_group(&ecx_context, IOmap, 0);

	if (ecx_slavecount == 0) {
		std::cout << "Found no EtherCAT slaves." << std::endl;
		return false;
	}
	
	for (int i=1; i<=ecx_slavecount; i++) {
		std::cout << "slave " << i << " has id = " << ecx_slave[i].eep_id << std::endl;
    }
	std::cout << "Found slaves: " << ecx_slavecount << ", " << *(ecx_context.slavecount) << std::endl;
	for (unsigned int i = 0; i < nWheels; i++) {
		int slave = (*wheelConfigs)[i].ethercatNumber;
		std::cout << "Wheel #" << i << " is slave #" << slave << std::endl; 
		if (slave > ecx_slavecount) { // slaves start index 1
			std::cout << "Found only " << ecx_slavecount << " EtherCAT slaves, but config requires at least " << slave << std::endl; 
			return false;
		}

		if (ecx_slave[slave].eep_id != 24137745 && ecx_slave[slave].eep_id != 0 && ecx_slave[slave].eep_id != 0x17010091 && ecx_slave[slave].eep_id != 0x02001001) {
			std::cout << "EtherCAT slave #" << i << " has wrong id: " << ecx_slave[slave].eep_id << std::endl;
			return false;
		}
	}

	for (unsigned int i = 0; i < modules.size(); i++) {
		bool ok = modules[i]->initEtherCAT(ecx_slave, ecx_slavecount);
		if (!ok)
			return false;
	}

	std::cout << ecx_slavecount << " EtherCAT slaves found and configured.\n";

	// wait for all slaves to reach SAFE_OP state */
	ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
	if (ecx_slave[0].state != EC_STATE_SAFE_OP) {
		std::cout << "Not all EtherCAT slaves reached safe operational state.\n";
		ecx_readstate(&ecx_context);

		// If not all slaves operational find out which one
		for (int i = 1; i <= ecx_slavecount; i++) {
			if (ecx_slave[i].state != EC_STATE_SAFE_OP) {
				std::cout << "Slave " << i << " State=" << ecx_slave[i].state << " StatusCode=" << ecx_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ecx_slave[i].ALstatuscode) << "\n";
			}
		}

		return false;
	}

	// set command to zero (specific for SmartWheel)
	rxpdo1_t data;
	data.timestamp = 1;
	data.command1 = 0;
	data.limit1_p = 0;
	data.limit1_n = 0;
	data.limit2_p = 0;
	data.limit2_n = 0;
	data.setpoint1 = 0;
	data.setpoint2 = 0;
	for (unsigned int i = 0; i < nWheels; i++) {
		rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
		*ecData = data;
	}

	ecx_send_processdata(&ecx_context);

	std::cout << "Request operational state for all EtherCAT slaves\n";

	// request OP state for all slaves
	ecx_slave[0].state = EC_STATE_OPERATIONAL;
	// send one valid process data to make outputs in slaves happy
	ecx_send_processdata(&ecx_context);
	ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
	// request OP state for all slaves
	ecx_writestate(&ecx_context, 0);

	// wait for all slaves to reach OP state
	ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
	if (ecx_slave[0].state == EC_STATE_OPERATIONAL) {
		std::cout << "Operational state reached for all EtherCAT slaves.\n";
	} else {
		std::cout << "Not all EtherCAT slaves reached operational state.\n";
		return false;
	}

	for (int cnt = 1; cnt <= ecx_slavecount; cnt++) {
		std::cout << "Slave: " << cnt  << " Name: " << ecx_slave[cnt].name  << " Output size: " << ecx_slave[cnt].Obits
			<< "bits Input size: " << ecx_slave[cnt].Ibits << "bits State: " << ecx_slave[cnt].state  
			<< " delay: " << ecx_slave[cnt].pdelay << std::endl; //<< " has dclock: " << (bool)ecx_slave[cnt].hasdc;
	}

	for (unsigned int i = 0; i < nWheels; i++) {
		processData[i] = *getProcessData((*wheelConfigs)[i].ethercatNumber);
	}
	lastProcessData = processData;

 	ethercatThread = new boost::thread(boost::bind(&PlatformDriver::ethercatHandler, this));

	return true;
}

void PlatformDriver::closeEthercat() {
	ecx_slave[0].state = EC_STATE_SAFE_OP;

	// request SAFE_OP state for all slaves */
	ecx_writestate(&ecx_context, 0);

	// stop SOEM, close socket
	ecx_close(&ecx_context);
}

void PlatformDriver::pauseEthercat(int ms) {
	pauseThreadMs = ms;
}

void PlatformDriver::printEthercatStatus() {
	ecx_readstate(&ecx_context);
	for (int i = 1; i <= ecx_slavecount; i++) {
		std::cout << "Slave " << i << " State=" << ecx_slave[i].state << " StatusCode=" << ecx_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ecx_slave[i].ALstatuscode) << std::endl;
	}
}

void PlatformDriver::reconnectSlave(int slave) {
	flagReconnectSlave = true;
}

void PlatformDriver::ethercatHandler() {
	int step = 0;
	threadPhase = 1;
	int wkc = 0;
	long timeToWait = 0;
	boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration pastTime;

	int counterIterations = 0;
	unsigned int ethercatTimeout = 1000;
	long int communicationErrors = 0;
	long int maxCommunicationErrors = 100;
	int timeTillNextEthercatUpdate = 1000; //usec
	
	if (state == DRIVER_STATE_INIT)
		state = DRIVER_STATE_READY;

	while (!stopThread) {
		threadPhase = 2;
		pastTime = boost::posix_time::microsec_clock::local_time() - startTime;
		timeToWait = timeTillNextEthercatUpdate - pastTime.total_microseconds();

		if (timeToWait < 0 || timeToWait > (int) timeTillNextEthercatUpdate) {
			//printf("Missed communication period of %d  microseconds it have been %d microseconds \n",timeTillNextEthercatUpdate, (int)pastTime.total_microseconds()+ 100);
		} else {
			boost::this_thread::sleep(boost::posix_time::microseconds(timeToWait));
		}

		if (pauseThreadMs > 0) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(pauseThreadMs));
			pauseThreadMs = 0;
		}

		// check for slave timeout errors
		ecx_readstate(&ecx_context); // TODO check return code
		for (unsigned int i = 0; i < nWheels; i++) {
			int slave = (*wheelConfigs)[i].ethercatNumber;
			if (ecx_slave[slave].state == EC_STATE_SAFE_OP + EC_STATE_ERROR) {
				std::cout << "Trying to reconnect slave " << slave << std::endl;
		
			//if (flagReconnectSlave) {	
				ecx_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
				ecx_writestate(&ecx_context, slave);
			  ecx_statecheck(&ecx_context, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
				ecx_readstate(&ecx_context);
			  if (ecx_slave[slave].state != EC_STATE_SAFE_OP) {
			    std::cout << "Failed to reset slave to SAFE_OP.\n";
				} else {
					ecx_slave[slave].state = EC_STATE_OPERATIONAL;
	
					/*rxpdo1_t data;
					data.timestamp = 1;
					data.command1 = 0;
					data.command2 = 0;
					data.limit1_p = 0;
					data.limit1_n = 0;
					data.limit2_p = 0;
					data.limit2_n = 0;
					data.setpoint1 = 0;
					data.setpoint2 = 0;
					for (unsigned int i = 0; i < nWheels; i++) {
						rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
						*ecData = data;
					}
					*/
					
				  ecx_send_processdata(&ecx_context);
				  ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
					ecx_writestate(&ecx_context, slave);
				  ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
					ecx_readstate(&ecx_context);
				  if (ecx_slave[slave].state != EC_STATE_OPERATIONAL) {
				    std::cout << "Failed to reset slave " << slave << " to OP.\n";
					} else {
				   std::cout << "Returned slave " << slave << " to OP state.\n";
				  }
				}
			//flagReconnectSlave = false;
			}
		}
			
		startTime = boost::posix_time::microsec_clock::local_time();

		threadPhase = 3;

		wkc = ecx_receive_processdata(&ecx_context, ethercatTimeout);

		// TODO fix
static int expectedWkc = 0;
static int lastWkc = 0;
//if (step < 10)
//	std::cout << "wkc=" << wkc << std::endl;

		if (wkc != lastWkc) {
			std::cout << "Smartwheel WKC changed from " << lastWkc << " to " << wkc << std::endl;
			lastWkc = wkc;
		}

		if (expectedWkc == 0) {
			if (wkc > 0) {
				std::cout << "Set new expected wkc=" << wkc << std::endl;
				expectedWkc = wkc;
			} else if (step > 0) {
				if (!ethercatWkcError)
					std::cout << "WKC error: expected " << expectedWkc << ", got " << wkc << ", step=" << step << std::endl;
				ethercatWkcError = true;				
			}
		} else if (wkc != expectedWkc) {
			if (!ethercatWkcError)
				std::cout << "WKC error: expected " << expectedWkc << ", got " << wkc << std::endl;
			ethercatWkcError = true;
		}
		
		if (wkc == 0) {
			threadPhase = 4;
			if (communicationErrors == 0) {
				std::cout << "Receiving data failed" << std::endl;
			}
			communicationErrors++;
		} else {
			threadPhase = 5;
			communicationErrors = 0;
		}

		if (communicationErrors > maxCommunicationErrors) {
			std::cout << "Lost EtherCAT connection" << std::endl;
			closeEthercat();
			stopThread = true;
			break;
		}

		threadPhase = 6;

		lastProcessData = processData;
		for (unsigned int i = 0; i < nWheels; i++) // TODO: use an array or vector of wheels or atleast use firstWheel
			processData[i] = *getProcessData((*wheelConfigs)[i].ethercatNumber);
		
		// TODO check if should take timestamp differently, or from each wheel separately
		current_ts = processData[0].sensor_ts; // TODO: atleast use firstWheel
		threadPhase = 7;
	
		if (step > 10) // TODO check
			updateStatusError();
		
		if (state == DRIVER_STATE_INIT) {
				state = DRIVER_STATE_READY;
		} else { // do control loop
			updateEncoders();
		
			if (!statusError && expectedWkc > 0 && state == DRIVER_STATE_READY) {
				if (canChangeActive)
					state = DRIVER_STATE_ACTIVE;
				else if (!showedMessageChangeActive) {
					std::cout << "platform driver is ready, but waiting for signal to become active." << std::endl;
					showedMessageChangeActive = true;
				}		
			}

			if (state == DRIVER_STATE_READY)
				doStop();
			else if (state == DRIVER_STATE_ACTIVE)
				doControl();
		}

		for (unsigned int i = 0; i < modules.size(); i++) {
			bool ok = modules[i]->step();
			// TODO react if not ok
		}

		//send and receive data from ethercat
		wkc = ecx_send_processdata(&ecx_context);
		if (wkc == 0) {
			std::cout << "Sending process data failed" << std::endl;
		}

		if (ecx_iserror(&ecx_context))
			std::cout << "there is an error in the soem driver" << std::endl;

		// TODO if ((startTime - timeLastSetMsgBuffer).total_milliseconds() > 1000) {
		// Stop robot due to timeout
		// }
		
		step++;
	}
	
	std::cout << "Stopped EtherCAT thread" << std::endl;
}

txpdo1_t* PlatformDriver::getProcessData(int slave) {
	// TODO: thread synchronization
	return (txpdo1_t*) ecx_slave[slave].inputs;
}

std::vector<double> PlatformDriver::getEncoderValue(int idx) {
	if (idx < 0 || idx >= nWheels)
		std::cout << "Failed to return encoder value. Encoder index does not match" << std::endl;
	else
		return sum_encoder[idx];
}

void PlatformDriver::setProcessData(int slave, rxpdo1_t* data) {
	// TODO: thread synchronization
	rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[slave].outputs;
	*ecData = *data;
}

const	WheelData* PlatformDriver::getWheelData(unsigned int wheel) {
	return &(*wheelData)[wheel];
}

void PlatformDriver::updateStatusError() {
	const int STATUS1a = 3;
	const int STATUS1b = 63;
	const int STATUS2 = 2051;
	
	bool error = false;
	int errors1 = 0;
	int errors2 = 0;
	unsigned int errorWheel = 0;
	
	for (unsigned int i = 0; i < nWheels; i++) {
		int s1 = processData[i].status1;
		int s2 = processData[i].status2;
		if ((s1 != STATUS1a && s1 != STATUS1b) || s2 != STATUS2) {
			error = true;
			errors1 = s1;
			errors2 = s2;
			errorWheel = i;
		}
	}

//			std::cout << "update ta " << error <<  std::endl;

	// TODO separate check for READY
	if (error && state != DRIVER_STATE_READY && state != DRIVER_STATE_INIT) {
		if (!statusError) {
			std::cout << "Status error: wheel=" << errorWheel << ", status1=" << errors1 << ", status2=" << errors2 << ", state=" << state << std::endl;
		}
		statusError = true;
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

	if (stallError)
		status |= DRIVER_ERROR_STALL;

	if (slipError)
		status |= DRIVER_ERROR_SLIP;
	
	return status;
}

void PlatformDriver::resetErrorFlags() {
	ethercatWkcError = false;
	statusError = false;
	stallError = false;
	slipError = false;
	timestampError = false;
}

int PlatformDriver::checkSmartwheelTimestamp() {
	bool swOK = true;
	bool criticalError = false;
	for (int i = 0; i < nWheels; i++) {
		bool error = false;
		bool critical = false;
		txpdo1_t* swData = getProcessData((*wheelConfigs)[i].ethercatNumber);
		if (wheel_sensor_ts[i] < swData->sensor_ts)
			wheel_sensor_ts[i] = swData->sensor_ts;
		else {
			error = true;
			if ((*wheelConfigs)[i].critical)
				critical = true;
		}
		if (wheel_setpoint_ts[i] < swData->setpoint_ts)
			wheel_setpoint_ts[i] = swData->setpoint_ts;
		else {
			error = true;
			if ((*wheelConfigs)[i].critical)
				critical = true;
		}
		
		if (error != (*wheelData)[i].errorTimestamp) {
			if (error)
				std::cout << "Timestamp error wheel " << i << ".\n";
			else
				std::cout << "Timestamp wheel " << i << " is ok again.\n";
			(*wheelData)[i].errorTimestamp = error;
		}

		swOK = swOK & !error;
		critical = critical | criticalError;
	}
	
	int result = 4;
	if (swOK)
		result = 2;
	else if (criticalError)
		result =  3;
		
	if (!swOK)
		timestampError = true;
	
	return result;		
}

void PlatformDriver::updateEncoders() {
	if(!encoderInitialized) {
		for (int i = 0; i < nWheels; i++) {
			txpdo1_t* wData = getProcessData((*wheelConfigs)[i].ethercatNumber);
			prev_encoder[i][0] = wData->encoder_1;
			prev_encoder[i][1] = wData->encoder_2;
		}
		encoderInitialized = true;
	}
	
	//count accumulative encoder value
	for (int i = 0; i < nWheels; i++) {
		txpdo1_t* wData = getProcessData((*wheelConfigs)[i].ethercatNumber);
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

	for (unsigned int i = 0; i < nWheels; i++) {
		rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
		*ecData = rxdata;
	}
}

void PlatformDriver::doControl() {
    // ASSUMPTION: firstWheel == 0 (TODO: discuss with Walter Nowak)
    assert ( firstWheel == 0 );

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
	
	for (size_t i = firstWheel; i < firstWheel + nWheels; i++) {
		txpdo1_t* wheel_data = getProcessData((*wheelConfigs)[i].ethercatNumber);

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
        rxpdo1_t* ecData = (rxpdo1_t*) ecx_slave[(*wheelConfigs)[i].ethercatNumber].outputs;
        *ecData = rxdata;					
    }
}

} //namespace kelo
