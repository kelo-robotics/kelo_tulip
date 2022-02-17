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


#ifndef MODULES_ROBILEMASTERBATTERY_H
#define MODULES_ROBILEMASTERBATTERY_H

#include "kelo_tulip/EtherCATModule.h"
#include <boost/thread.hpp>

namespace kelo {

class RobileMasterBattery : public EtherCATModule {
public:
	RobileMasterBattery(int slaveNumber);
	virtual ~RobileMasterBattery();
	
	bool initEtherCAT(ec_slavet* ecx_slaves, int ecx_slavecount);
	bool step();
	
	void shutdown(int seconds);
	void startCharge();
	void stopCharge();
	
	void pump(bool enablePump);
	void dock(bool dock);
	void undock(bool undock);
	
private:
	ec_slavet* ecx_slaves;
	int slaveNumber;
	
	bool flagShutdown;
	bool robileCharge;
	bool robileEnablePump;
	bool robileEnableDock;
	bool robileEnableUndock;		

	boost::posix_time::ptime pumpStartTime;
	boost::posix_time::ptime dockStartTime;
	boost::posix_time::ptime undockStartTime;
};

struct __attribute__((packed)) RobileMasterBatteryProcessDataInput {
	uint64_t TimeStamp;  	 	 // EtherCAT timestamp ms
	uint16_t Status;     	 	 // Status bits
	uint16_t Error;					 // Error bits: bit0:LowBAT,bit1:EMPTYFRAM,bit2:WrongFRAM,bit3:WrongCHGARGER,bit4:OverloadCHG,bit5:OverCurrentError,bit6:Watchdog
	uint16_t Warning;				 // Warning bits:bit0:OverCurrentWarning,bit1:Shutdown
	float    OutputCurrent;  // Total current consumption
	float    OutputVoltage;  // System Voltage
	float    OutputPower;    // Total power consumption of the system
	float    AuxPortCurrent; // Current consumption at Auxiliary port
	float    GenericData1; 	 // Generic data, might be used for different purposes
	uint32_t  GenericData2;	 // Generic data, might be used for different purposes
	
	uint16_t bmsm_PwrDeviceId;
	uint16_t bmsm_Status;
	float    bmsm_Voltage;
	float    bmsm_Current;
	float    bmsm_Temperature;
	uint16_t bmsm_SOC;
	uint32_t bmsm_SN;
	uint32_t bmsm_BatData1;
	float    bmsm_BatData2;
};

struct __attribute__((packed)) RobileMasterBatteryProcessDataOutput {
  uint32_t      Command1;
  uint32_t      Command2;
  uint16_t      Shutdown;
  uint16_t      PwrDeviceId;
};

} // namespace kelp

#endif // MODULES_ROBILEMASTERBATTERY_H
