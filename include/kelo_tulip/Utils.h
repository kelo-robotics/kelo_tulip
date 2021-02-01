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



#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <vector>
#include <string>

namespace kelo
{

    class Utils
    {
        public:
            /**
             * @brief: clip `value` to be in range of `min_limit` to `max_limit`
             */
            static float clip(float value, float max_limit, float min_limit);

            /**
             * @brief: clip angle to be in range of -pi to pi
             */
            static float clipAngle(float angle);


            /**
             * @brief: Calculate shortest angular distance between 2 angles. For
             * example, the shortest angular distance between 0 degrees and 270
             * degrees is -90 degrees (instead of 270 degrees). Note: all angles
             * are in radians.
             */
            static float getShortestAngle(float angle1, float angle2);

    };

} /* namespace kelo */

#endif /* UTILS_H */
