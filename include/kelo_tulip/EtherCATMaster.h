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


#ifndef KELOTULIP_ETHERCATMASTER_H_
#define KELOTULIP_ETHERCATMASTER_H_

extern "C" {
#include "kelo_tulip/soem/ethercattype.h"
#include "kelo_tulip/EtherCATModule.h"

#include "nicdrv.h"
#include "kelo_tulip/soem/ethercatbase.h"
#include "kelo_tulip/soem/ethercatmain.h"
#include "kelo_tulip/soem/ethercatconfig.h"
#include "kelo_tulip/soem/ethercatcoe.h"
#include "kelo_tulip/soem/ethercatdc.h"
#include "kelo_tulip/soem/ethercatprint.h"
}
#include <boost/thread.hpp>
#include <string>
#include <fstream>

namespace kelo {

class EtherCATMaster {
public:
	EtherCATMaster(std::string device, std::vector<EtherCATModule*> modules);
	virtual ~EtherCATMaster();

	bool initEthercat();
	void closeEthercat();
	void pauseEthercat(int ms);
	void printEthercatStatus();
	void reconnectSlave(int slave);

	bool hasWkcError();
	void resetErrorFlags();

protected:
	void ethercatHandler();
	void ethercatCheck();
	std::ofstream logfile;
	bool canChangeActive;
	bool showedMessageChangeActive;

	ec_slavet ecx_slave[EC_MAXSLAVE];
	int ecx_slavecount;
	ec_groupt ec_group[EC_MAXGROUP];
	uint8 esibuf[EC_MAXEEPBUF];
	uint32 esimap[EC_MAXEEPBITMAP];
	ec_eringt ec_elist;
	ec_idxstackT ec_idxstack;
	ec_SMcommtypet ec_SMcommtype;
	ec_PDOassignt ec_PDOassign;
	ec_PDOdesct ec_PDOdesc;
	ec_eepromSMt ec_SM;
	ec_eepromFMMUt ec_FMMU;
	boolean EcatError;
	int64 ec_DCtime;			   
	ecx_portt ecx_port;
	ecx_redportt ecx_redport;
	ecx_contextt ecx_context;
	int expectedWKC;
	volatile int wkc;
	bool inOP = false;
	
	std::vector<EtherCATModule*> modules;

	char IOmap[4096];
	std::string device;
	bool ethercatInitialized;
	boost::thread* ethercatThread;
	boost::thread* ethercatCheckThread;
	volatile bool stopThread;
	volatile int threadPhase;
	volatile int pauseThreadMs;

	volatile bool ethercatWkcError;
	volatile bool flagReconnectSlave;

private:
	EtherCATMaster(const EtherCATMaster&);
};

} //namespace kelo

#endif //KELOTULIP_ETHERCATMASTER
