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


#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>
#include <string>
#include <iostream>

namespace kelo
{

    /* ---------------------------------------------------------------------- */

    struct Point2D
    {
        float x, y;

        friend std::ostream& operator << (std::ostream &out, const Point2D& pt)
        {
            out << "<x: " << pt.x
                << ", y: " << pt.y
                << ">";
            return out;
        }
    };

    /* ---------------------------------------------------------------------- */

    struct Attitude2D
    {
        float x, y, a;

        friend std::ostream& operator << (std::ostream &out, const Attitude2D& attitude)
        {
            out << "<x: " << attitude.x
                << ", y: " << attitude.y
                << ", a: " << attitude.a
                << ">";
            return out;
        }
    };

    /* ---------------------------------------------------------------------- */

    struct Attitude3D
    {
        float x, y, z;

        /* rotation definition is right hand rule */
        float rx, ry, rz;

        friend std::ostream& operator << (std::ostream &out, const Attitude3D& attitude)
        {
            out << "<x: " << attitude.x
                << ", y: " << attitude.y
                << ", z: " << attitude.z
                << ", rx: " << attitude.rx
                << ", ry: " << attitude.ry
                << ", rz: " << attitude.rz
                << ">";
            return out;
        }
    };

    /* ---------------------------------------------------------------------- */

    struct WheelParamVelocity
    {
        Point2D pivot_position;             // pivot location relative to vehicle centre
        float   pivot_offset;               // pivot offset relative to vehicle direction of travel
        Point2D relative_position_l;        // location of left wheel relative to pivot
        Point2D relative_position_r;        // location of right wheel relative to pivot
        float   linear_to_angular_velocity; // scaling m/s to rad/s
        float   angular_to_linear_velocity; // scaling rad/s to m/s
        float   max_linear_velocity;        // maximum velocity of wheel
        float   max_pivot_error;            // maximum pivot error of smart wheel used for error correction
        float   pivot_kp;                   // proportional gain for pivot position controller
        float   wheel_diameter;             // wheel diameter

        friend std::ostream& operator << (std::ostream &out, const WheelParamVelocity& wp)
        {
            out << "<" << std::endl
                << " pivot_position: " << wp.pivot_position << std::endl
                << " pivot_offset: " << wp.pivot_offset << std::endl
                << " relative_position_l: " << wp.relative_position_l << std::endl
                << " relative_position_r: " << wp.relative_position_r << std::endl
                << " linear_to_angular_velocity: " << wp.linear_to_angular_velocity << std::endl
                << " angular_to_linear_velocity: " << wp.angular_to_linear_velocity << std::endl
                << " max_linear_velocity: " << wp.max_linear_velocity << std::endl
                << " max_pivot_error: " << wp.max_pivot_error << std::endl
                << " pivot_kp: " << wp.pivot_kp << std::endl
                << " wheel_diameter: " << wp.wheel_diameter << std::endl
                << ">";
            return out;
        }
    };

    /* ---------------------------------------------------------------------- */

} /* namespace kelo */

#endif /* STRUCTS_H */
