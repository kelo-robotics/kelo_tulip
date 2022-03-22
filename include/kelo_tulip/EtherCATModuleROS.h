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

#ifndef ETHERCATMODULEROS_H
#define ETHERCATMODULEROS_H

#include "kelo_tulip/EtherCATModule.h"
#include <ros/ros.h>

namespace kelo {

//! Base class for ROS interface of a module.
//! This class should create a new EtherCATModule and configure it.
//! The created module will then be passed on to the EtherCAT master,
//! while this ROS module can provide external access via ROS.

class EtherCATModuleROS {
public:
	//! Default constructor.
	EtherCATModuleROS();
	
	//! Default destructor.
	virtual ~EtherCATModuleROS();

	//! Initialize this module, must be overridden.
	//! Returns true if module could be successfully initialized.
	virtual bool init(ros::NodeHandle& nh) = 0;

	//! Function that is continously called by ROS main loop to publish or process data.
	//! Returns true if module can continue to run.
	virtual bool step();

	//! Return the type of this module as string.
	virtual std::string getType() = 0;

	//! Return internal EtherCAT module, must be overridden.
	virtual EtherCATModule* getEtherCATModule() = 0;

protected:
	//! Global NodeHandle, by default set in init() function.
	ros::NodeHandle nh;
};

} // namespace kelp

#endif // ETHERCATMODULEROS_H
