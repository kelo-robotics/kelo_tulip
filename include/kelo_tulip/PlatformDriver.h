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


#ifndef PLATFORMDRIVER_H_
#define PLATFORMDRIVER_H_

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "kelo_tulip/EtherCATModule.h"
#include "kelo_tulip/KeloDriveAPI.h"

#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}
#include "kelo_tulip/VelocityPlatformController.h"
#include "kelo_tulip/Utils.h"
#include "kelo_tulip/WheelConfig.h"
#include <boost/thread.hpp>
#include <string>
#include <fstream>

namespace kelo {

struct WheelData {
	bool enable;
	
	bool error;
	bool errorTimestamp;

};

enum DriverState {
	DRIVER_STATE_UNDEFINED = 0x00,
	DRIVER_STATE_ACTIVE = 0x01,
	DRIVER_STATE_READY = 0x02,
	DRIVER_STATE_INIT = 0x04,
	DRIVER_STATE_ERROR = 0x10,
	DRIVER_STATE_TIMEOUT = 0x20
};

enum DriverError {
	DRIVER_ERROR_UNSPECIFIED = 0x0100,
	DRIVER_ERROR_ETHERCAT_WKC = 0x0200,
	DRIVER_ERROR_TIMESTAMP = 0x0400,
	DRIVER_ERROR_STATUS = 0x0800,
	DRIVER_ERROR_STALL = 0x1000,
	DRIVER_ERROR_SLIP = 0x2000
};

class PlatformDriver : public EtherCATModule {
public:
	PlatformDriver(const std::vector<WheelConfig>& wheelConfigs, const std::vector<WheelData>& wheelData);
	virtual ~PlatformDriver();

	virtual bool initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount);
	virtual bool step();

	txpdo1_t* getWheelProcessData(unsigned int wheel);
	void setWheelProcessData(unsigned int wheel, rxpdo1_t* data);

	void reconnectSlave(int slave);

	void setTargetVelocity(double vx, double vy, double va);

	void setWheelDistance(double x);
	void setWheelDiameter(double x);
	void setCurrentStop(double x);
	void setCurrentDrive(double x);
	double getCurrentDrive();

	void setCanChangeActive();

	void setMaxvlin(double x);
	double getMaxvlin();
	void setMaxva(double x);
	double getMaxva();
	void setMaxvlinacc(double x);
	void setMaxangleacc(double x);
	void setMaxvaacc(double x);
	void setFactorAngleaccVlin(double x);
	void setFractionVelTolerance(double x);
	void setFractionFactor(double x);

	void setVheadingControlp(double x);
	void setMaxvheading(double x);
	void setWheelsetpointacc(double x);
	void setWheelsetpointmin(double x);

	const	WheelData* getWheelData(unsigned int wheel);
	
	int getDriverStatus();

	void resetErrorFlags();

	std::vector<double> getEncoderValue(int idx);
		
protected:
	int checkSmartwheelTimestamp();
	void updateEncoders();
	void updateSetpoints();
	virtual void doStop();
	virtual void doControl();

	bool hasWheelStatusEnabled(unsigned int wheel);
	bool hasWheelStatusError(unsigned int wheel);
	virtual void updateStatusError();

	volatile DriverState state;
	std::ofstream logfile;
	bool canChangeActive;
	bool showedMessageChangeActive;
	int stepCount;

	ec_slavet* ecx_slaves;
	
	std::vector<EtherCATModule*> modules;

	std::vector<txpdo1_t> processData;
	std::vector<txpdo1_t> lastProcessData;
	long unsigned int current_ts;
	std::vector<long unsigned int> wheel_setpoint_ts;
	std::vector<long unsigned int> wheel_sensor_ts;
	unsigned int swErrorCount;
	double curr_setpoint1, curr_setpoint2;
	std::vector<std::vector<double> > prev_encoder;
	std::vector<std::vector<double> > sum_encoder;
	bool encoderInitialized;
	volatile bool ethercatWkcError;
	volatile bool flagReconnectSlave;

	int firstWheel, nWheels;
	std::vector<WheelConfig> wheelConfigs;
	std::vector<WheelData> wheelData;

	double wheelDistance;
	double wheelDiameter;
	
	double maxCalibrationTime;
	double vCalibration;
	double currentCalibration;
	double currentStop;
	double currentDrive;

	double maxvlin;
	double maxva;
	double maxvlinacc;
	double maxangleacc;
	double maxvaacc;

	double wheelsetpointmin;
	double wheelsetpointmax;

	volatile bool statusError;
	volatile bool timestampError;

private:
	PlatformDriver(const PlatformDriver&);
	VelocityPlatformController velocityPlatformController;
};

} //namespace kelo

#endif //PLATFORM_DRIVER_H_
