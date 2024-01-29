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
#include <iostream>

namespace kelo {

EtherCATMaster::EtherCATMaster(std::string device, std::vector<EtherCATModule*> modules)
	: device(device)
	, modules(modules)
{
	ethercatInitialized = false;
	threadPhase = 0;
	pauseThreadMs = 0;
	flagReconnectSlave = false;
	expectedWKC = 0;

	EcatError = FALSE;
	ethercatWkcError = false;

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
}

EtherCATMaster::~EtherCATMaster() {
}

EtherCATMaster::EtherCATMaster(const EtherCATMaster&) {
}

bool EtherCATMaster::initEthercat() {
	if (!ethercatInitialized) {
		if (!ecx_init(&ecx_context, const_cast<char*>(device.c_str()))) {
			std::cout << "Failed to initialize EtherCAT on " << device << " with communication thread\n";
			return false;
		}
		std::cout << "Initializing EtherCAT on " << device << "\n";
		
		ethercatInitialized = true;
	}

	// find and auto-config slaves
	int wkc2 = ecx_config_init(&ecx_context, TRUE);
	if (!wkc2) {
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

	// determine expected WKC
	expectedWKC = (ecx_context.grouplist[0].outputsWKC * 2) + ecx_context.grouplist[0].inputsWKC;

	// initialize modules and return false if any fails	
	std::cout << "Modules to init : " << modules.size() << std::endl;
	for (unsigned int i = 0; i < modules.size(); i++) {
		std::cout << " Module " << i << " initializing..." << std::endl;
		bool ok = modules[i]->initEtherCAT(ecx_slave, ecx_slavecount);
		if (!ok) {
			std::cout << "Module fails, stop EtherCATMaster." << std::endl;
			return false;
		}
		std::cout << " Module " << i << " Result " << ok << std::endl;
		ok = modules[i]->initEtherCAT2(&ecx_context, ecx_slavecount);
	}
	std::cout << "Module init finished." << std::endl;

	std::cout << ecx_slavecount << " EtherCAT slaves found and configured." << std::endl;

	// wait for all slaves to reach SAFE_OP state */
	ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
	if (ecx_slave[0].state != EC_STATE_SAFE_OP) {
		std::cout << "Not all EtherCAT slaves reached safe operational state." << std::endl;
		ecx_readstate(&ecx_context);

		// If not all slaves operational find out which one
		for (int i = 1; i <= ecx_slavecount; i++) {
			if (ecx_slave[i].state != EC_STATE_SAFE_OP) {
				std::cout << "Slave " << i << " State=" << ecx_slave[i].state << " StatusCode=" << ecx_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ecx_slave[i].ALstatuscode) << std::endl;
			}
		}

		return false;
	}

	// request OP state for all slaves
	std::cout << "Request operational state for all EtherCAT slaves\n";
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


	wkc = expectedWKC;
 	ethercatCheckThread = new boost::thread(boost::bind(&EtherCATMaster::ethercatCheck, this));

	boost::thread::attributes attrs;
	sched_param param;
	int ret;
	ret = pthread_attr_init(attrs.native_handle());
	attrs.set_stack_size(4096*32);
	ret = pthread_attr_getschedparam(attrs.native_handle(), &param);
	param.sched_priority = 40;
    ret = pthread_attr_setschedpolicy(attrs.native_handle(), SCHED_FIFO);
	ret = pthread_attr_setschedparam (attrs.native_handle(), &param);

 	ethercatThread = new boost::thread(attrs, boost::bind(&EtherCATMaster::ethercatHandler, this));
// 	ethercatThread = new boost::thread(boost::bind(&EtherCATMaster::ethercatHandler, this));
	inOP = true;

	return true;
}

void EtherCATMaster::closeEthercat() {
	ecx_slave[0].state = EC_STATE_SAFE_OP;

	// request SAFE_OP state for all slaves */
	ecx_writestate(&ecx_context, 0);

	// stop SOEM, close socket
	ecx_close(&ecx_context);
}

void EtherCATMaster::pauseEthercat(int ms) {
	pauseThreadMs = ms;
}

void EtherCATMaster::printEthercatStatus() {
	ecx_readstate(&ecx_context);
	for (int i = 1; i <= ecx_slavecount; i++) {
		std::cout << "Slave " << i << " State=" << ecx_slave[i].state << " StatusCode=" << ecx_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ecx_slave[i].ALstatuscode) << std::endl;
	}
}

void EtherCATMaster::reconnectSlave(int slave) {
	flagReconnectSlave = true;
}

void EtherCATMaster::ethercatHandler() {
	int step = 0;
	int wkc2 = 0;
	long timeToWait = 0;
	boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration pastTime;

	unsigned int ethercatTimeout = 1000;
	long int communicationErrors = 0;
	long int maxCommunicationErrors = 1000;
	int timeTillNextEthercatUpdate = 1000; //usec

	ecx_send_processdata(&ecx_context);
	
	while (!stopThread) {
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

		startTime = boost::posix_time::microsec_clock::local_time();

		wkc = ecx_receive_processdata(&ecx_context, ethercatTimeout);

		if (wkc != expectedWKC) {
			if (!ethercatWkcError)
				std::cout << "WKC error: expected " << expectedWKC << ", got " << wkc << std::endl;
			ethercatWkcError = true;
		} else ethercatWkcError = false;
		
		if (wkc <= 0) {
			if (communicationErrors == 0) {
				std::cout << "Receiving data failed" << std::endl;
			}
			communicationErrors++;
		} else {
			communicationErrors = 0;
		}

		if (communicationErrors == maxCommunicationErrors) {
			std::cout << "Lost EtherCAT connection" << std::endl;
		}

		// check for slave timeout errors
		static uint32_t cnt = 0;
		if((cnt++ % 64) == 0)
			ecx_readstate(&ecx_context);

		bool modulesOK = true;
		for (unsigned int i = 0; i < modules.size() && modulesOK; i++)
			modulesOK = modules[i]->step();

		if (!modulesOK) {
			std::cout << "EtherCAT module failed, stopping EtherCAT communication." << std::endl;
			closeEthercat();
			stopThread = true;
			break;
		}

		//send and receive data from ethercat
		wkc2 = ecx_send_processdata(&ecx_context);
		if (wkc2 == 0) {
			std::cout << "Sending process data failed" << std::endl;
		}

		if (ecx_iserror(&ecx_context)) {
			ec_errort ec;
			ecx_poperror(&ecx_context, &ec);
			std::cout << "There is an error in the soem driver" << std::endl;
		}
		
		step++;
	}
	
	std::cout << "Stopped EtherCAT thread" << std::endl;
}

#define EC_TIMEOUTMON 500

void EtherCATMaster::ethercatCheck(void)
{
    int slave;
	int currentgroup = 0;
	std::cout << "Start EtherCAT Check thread" << std::endl;

    while(!stopThread)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ecx_context);
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ecx_slave[slave].group == currentgroup) && (ecx_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ecx_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ecx_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ecx_writestate(&ecx_context, slave);
                  }
                  else if(ecx_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ecx_slave[slave].state = EC_STATE_OPERATIONAL;
                     ecx_writestate(&ecx_context, slave);
                  }
                  else if(ecx_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ecx_reconfig_slave(&ecx_context, slave, EC_TIMEOUTMON) >= EC_STATE_PRE_OP)
                     {
                        ecx_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ecx_slave[slave].islost)
                  {
                     /* re-check state */
                     ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ecx_slave[slave].state == EC_STATE_NONE)
                     {
                        ecx_slave[slave].islost = TRUE;
                        /* zero input data for this slave */
                        if(ecx_slave[slave].Ibytes)
                        {
                           memset(ecx_slave[slave].inputs, 0x00, ecx_slave[slave].Ibytes);
                           printf("zero inputs %p %d\n\r", ecx_slave[slave].inputs, ecx_slave[slave].Ibytes);
                        }
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ecx_slave[slave].islost)
               {
                  if(ecx_slave[slave].state <= EC_STATE_INIT)
                  {
                     if (ecx_recover_slave(&ecx_context, slave, EC_TIMEOUTMON))
                     {
                        ecx_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ecx_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
	std::cout << "Stopped EtherCAT Check thread" << std::endl;
}

bool EtherCATMaster::hasWkcError() {
	return ethercatWkcError;
}

void EtherCATMaster::resetErrorFlags() {
	ethercatWkcError = false;
}

} //namespace kelo
