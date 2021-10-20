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



#ifndef VELOCITY_PLATFORM_CONTROLLER_H
#define VELOCITY_PLATFORM_CONTROLLER_H

#include <kelo_tulip/Utils.h>
#include <kelo_tulip/Structs.h>
#include <kelo_tulip/WheelConfig.h>
#include <boost/thread.hpp>
#include <iostream>

namespace kelo
{

    class VelocityPlatformController
    {

        public:
            VelocityPlatformController();

            virtual ~VelocityPlatformController();

            void initialise(const std::vector<WheelConfig>& wheel_configs);

            void setPlatformMaxVelocity(float max_vel_linear, float max_vel_angular);
            void setPlatformMaxAcceleration(float max_acc_linear, float max_acc_angular);
            void setPlatformMaxDeceleration(float max_dec_linear, float max_dec_angular);
            
            void calculatePlatformRampedVelocities();

            void calculateWheelTargetVelocity(const size_t &wheel_index,
                                              const float &pivot_angle,
                                              float &target_ang_vel_l,
                                              float &target_ang_vel_r);

            void setPlatformTargetVelocity(const float &vel_x,
                                           const float &vel_y,
                                           const float &vel_a);

            friend std::ostream& operator << (std::ostream &out,
                                              const VelocityPlatformController& controller);

        private:

            std::vector<WheelParamVelocity> wheel_params_;

            Attitude2D platform_target_vel_;
            Attitude2D platform_ramped_vel_;           

        	  struct PlatformLimits {
                float max_vel_linear;
                float max_vel_angular;
                float max_acc_linear;
                float max_acc_angular;
                float max_dec_linear;
                float max_dec_angular;
        	  } platform_limits_;

        	  boost::posix_time::ptime time_last_ramping;
        	  bool first_ramping_call;
    };

} /* namespace kelo */

#endif /* VELOCITY_PLATFORM_CONTROLLER_H */
