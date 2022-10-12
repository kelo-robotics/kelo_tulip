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


#ifndef WHEELMODEL_H
#define WHEELMODEL_H

#include <vector>
#include <string>

namespace kelo {

struct WheelModel {
	std::string name;
	bool active;             // actively controlled or passive
	double diameter;
	double width;
	double casteroffset;
	double wheeldistance;    // [m] distance between centers of both hubwheels
	bool canPivot;
	double velocitylimit;    // [rad/s] for one hubwheel
	double currentlimit;
	
	WheelModel() {
		name = "KELOdrive105";
		active = true;
		diameter = 0.105;
		width = 0.040;
		casteroffset = 0.010;
		wheeldistance = 0.080;
		canPivot = true;
		velocitylimit = 100.0;
		currentlimit = 10.0;
	}
};

} // namespace kelp

#endif // WHEELMODEL_H
