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


#include <kelo_tulip/Utils.h>
#include <iostream>

namespace kelo
{

    float Utils::clip(float value, float max_limit, float min_limit)
    {
        return std::max(std::min(value, max_limit), min_limit);
    }

    float Utils::clipAngle(float angle)
    {
        if ( angle < -M_PI )
        {
            return angle + (2 * M_PI);
        }
        else if ( angle > M_PI )
        {
            return angle - (2 * M_PI);
        }
        else
        {
            return angle;
        }
    }

    float Utils::getShortestAngle(float angle1, float angle2)
    {
        return atan2(sin(angle1 - angle2), cos(angle1 - angle2));
    }

} /* namespace kelo */
