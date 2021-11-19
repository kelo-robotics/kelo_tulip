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


#include <kelo_tulip/VelocityPlatformController.h>
#include <math.h>

namespace kelo
{

    VelocityPlatformController::VelocityPlatformController()
    {
        platform_target_vel_.x = 0.0f;
        platform_target_vel_.y = 0.0f;
        platform_target_vel_.a = 0.0f;

        platform_ramped_vel_.x = 0.0f;
        platform_ramped_vel_.y = 0.0f;
        platform_ramped_vel_.a = 0.0f;
        
        // set default limits
        platform_limits_.max_vel_linear = 1.0;
        platform_limits_.max_vel_angular = 1.0;
        platform_limits_.max_acc_linear = 0.5;
        platform_limits_.max_acc_angular = 0.8;
        platform_limits_.max_dec_linear = 0.5;
        platform_limits_.max_dec_angular = 0.8;
        
        first_ramping_call = true;
    }

    VelocityPlatformController::~VelocityPlatformController()
    {
    }

    void VelocityPlatformController::setPlatformTargetVelocity(
            const float &vel_x,
            const float &vel_y,
            const float &vel_a)
    {
        platform_target_vel_.x = ( fabs(vel_x) < 0.0000001 ) ? 0.0 : vel_x;
        platform_target_vel_.y = ( fabs(vel_y) < 0.0000001 ) ? 0.0 : vel_y;
        platform_target_vel_.a = ( fabs(vel_a) < 0.0000001 ) ? 0.0 : vel_a;
    }

    void VelocityPlatformController::initialise(const std::vector<WheelConfig>& wheel_configs)
    {
        // FIXME: remove hardcoding wheel params

        unsigned int num_of_wheels = wheel_configs.size();

        float wheel_diameter = 0.105f;
        float wheel_caster = 0.01f;
        float wheel_distance = 0.055f;

        wheel_params_.clear();

        for ( size_t i = 0; i < num_of_wheels; i++ )
        {
            WheelParamVelocity wheel_param;
            wheel_param.relative_position_l.x = -1 * wheel_caster;
            wheel_param.relative_position_l.y = 0.5 * wheel_distance;
            wheel_param.relative_position_r.x = -1 * wheel_caster;
            wheel_param.relative_position_r.y = -0.5 * wheel_distance;
            wheel_param.angular_to_linear_velocity = 0.5 * wheel_diameter;
            wheel_param.linear_to_angular_velocity = 1.0 / wheel_param.angular_to_linear_velocity;
            wheel_param.max_linear_velocity = 100.0 * wheel_param.angular_to_linear_velocity;
            wheel_param.pivot_kp = 0.2f;
            wheel_param.wheel_diameter = wheel_diameter;
            wheel_param.max_pivot_error = M_PI * 0.25f;

	          wheel_param.pivot_position.x = wheel_configs[i].x;
	          wheel_param.pivot_position.y = wheel_configs[i].y;
	          wheel_param.pivot_offset = wheel_configs[i].a;

            wheel_params_.push_back(wheel_param);
        }
    }

    void VelocityPlatformController::setPlatformMaxVelocity(float max_vel_linear, float max_vel_angular)
    {
    	platform_limits_.max_vel_linear = max_vel_linear;
    	platform_limits_.max_vel_angular = max_vel_angular;
    }

    void VelocityPlatformController::setPlatformMaxAcceleration(float max_acc_linear, float max_acc_angular)
    {
    	platform_limits_.max_acc_linear = max_acc_linear;
    	platform_limits_.max_acc_angular = max_acc_angular;
    }

    void VelocityPlatformController::setPlatformMaxDeceleration(float max_dec_linear, float max_dec_angular)
    {
    	platform_limits_.max_dec_linear = max_dec_linear;
    	platform_limits_.max_dec_angular = max_dec_angular;
    }
            
    void VelocityPlatformController::calculatePlatformRampedVelocities()
    {
        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
        	
        // if this is called the first time, calculating time delta that way makes no sense
        if (first_ramping_call) {
            first_ramping_call = false;
            time_last_ramping = now;
            return;
        }
        	
        float time_delta = (now - time_last_ramping).total_microseconds() / 1000000.0f;

        // velocity ramps
        if (platform_ramped_vel_.x >= 0) {
            platform_ramped_vel_.x = Utils::clip(platform_target_vel_.x,
                platform_ramped_vel_.x + time_delta * platform_limits_.max_acc_linear,
                platform_ramped_vel_.x - time_delta * platform_limits_.max_dec_linear
            );
        } else {
            platform_ramped_vel_.x = Utils::clip(platform_target_vel_.x,
                platform_ramped_vel_.x + time_delta * platform_limits_.max_dec_linear,
                platform_ramped_vel_.x - time_delta * platform_limits_.max_acc_linear
            );
        }

        if (platform_ramped_vel_.y >= 0) {
            platform_ramped_vel_.y = Utils::clip(platform_target_vel_.y,
                platform_ramped_vel_.y + time_delta * platform_limits_.max_acc_linear,
                platform_ramped_vel_.y - time_delta * platform_limits_.max_dec_linear
            );
        } else {
            platform_ramped_vel_.y = Utils::clip(platform_target_vel_.y,
                platform_ramped_vel_.y + time_delta * platform_limits_.max_dec_linear,
                platform_ramped_vel_.y - time_delta * platform_limits_.max_acc_linear
            );
        }

        if (platform_ramped_vel_.a >= 0) {
            platform_ramped_vel_.a = Utils::clip(platform_target_vel_.a,
                platform_ramped_vel_.a + time_delta * platform_limits_.max_acc_angular,
                platform_ramped_vel_.a - time_delta * platform_limits_.max_dec_angular
            );
        } else {
            platform_ramped_vel_.a = Utils::clip(platform_target_vel_.a,
                platform_ramped_vel_.a + time_delta * platform_limits_.max_dec_angular,
                platform_ramped_vel_.a - time_delta * platform_limits_.max_acc_angular
            );
        }
        
        // velocity limits
        platform_ramped_vel_.x = Utils::clip(platform_ramped_vel_.x, platform_limits_.max_vel_linear, -platform_limits_.max_vel_linear);
        platform_ramped_vel_.y = Utils::clip(platform_ramped_vel_.y, platform_limits_.max_vel_linear, -platform_limits_.max_vel_linear);
        platform_ramped_vel_.a = Utils::clip(platform_ramped_vel_.a, platform_limits_.max_vel_angular, -platform_limits_.max_vel_angular);
      	
        time_last_ramping = now;
    }

    void VelocityPlatformController::calculateWheelTargetVelocity(
            const size_t &wheel_index,
            const float &raw_pivot_angle,
            float &target_ang_vel_l,
            float &target_ang_vel_r)
    {
        /* command 0 angular vel when platform has been commanded 0 vel
         * If this is not done, then the wheels pivot to face front of platform
         * even when the platform is commanded zero velocity.
         */
        if ( platform_ramped_vel_.x == 0 && platform_ramped_vel_.y == 0 && platform_ramped_vel_.a == 0 )
        {
            target_ang_vel_l = 0.0f;
            target_ang_vel_r = 0.0f;
            return;
        }

        const WheelParamVelocity &wheel_param = wheel_params_[wheel_index];

        /* pivot angle w.r.t. front of platform (between -pi and pi) */
        float pivot_angle = Utils::clipAngle(raw_pivot_angle
                                                      - wheel_param.pivot_offset);

        /* pivot angle to unity vector */
        Point2D unit_pivot_vector;
        unit_pivot_vector.x = cos(pivot_angle);
        unit_pivot_vector.y = sin(pivot_angle); 

        /* position of wheels relative to platform centre */
        Point2D position_l, position_r;
        position_l.x = (wheel_param.relative_position_l.x * unit_pivot_vector.x
                        - wheel_param.relative_position_l.y * unit_pivot_vector.y)
                       + wheel_param.pivot_position.x;
        position_l.y = (wheel_param.relative_position_l.x * unit_pivot_vector.y
                        + wheel_param.relative_position_l.y * unit_pivot_vector.x)
                       + wheel_param.pivot_position.y;
        position_r.x = (wheel_param.relative_position_r.x * unit_pivot_vector.x
                        - wheel_param.relative_position_r.y * unit_pivot_vector.y)
                       + wheel_param.pivot_position.x;
        position_r.y = (wheel_param.relative_position_r.x * unit_pivot_vector.y
                        + wheel_param.relative_position_r.y * unit_pivot_vector.x)
                       + wheel_param.pivot_position.y;

        /* velocity target vector at pivot position */
        Point2D target_vel_at_pivot;
        target_vel_at_pivot.x = platform_ramped_vel_.x
                                - (platform_ramped_vel_.a * wheel_param.pivot_position.y);
        target_vel_at_pivot.y = platform_ramped_vel_.y
                                + (platform_ramped_vel_.a * wheel_param.pivot_position.x);

        /* target pivot vector to angle */
        float target_pivot_angle = atan2(target_vel_at_pivot.y, target_vel_at_pivot.x);

        /* calculate error pivot angle as shortest route */
        float pivot_error = Utils::getShortestAngle(target_pivot_angle,
                                                             pivot_angle);

        /* limit pivot velocity */
        pivot_error = Utils::clip(pivot_error,
                                           wheel_param.max_pivot_error,
                                           -wheel_param.max_pivot_error);

        /* target velocity vector at wheel position */
        Point2D target_vel_vec_l, target_vel_vec_r;
        target_vel_vec_l.x = platform_ramped_vel_.x - (platform_ramped_vel_.a * position_l.y);
        target_vel_vec_l.y = platform_ramped_vel_.y + (platform_ramped_vel_.a * position_l.x);
        target_vel_vec_r.x = platform_ramped_vel_.x - (platform_ramped_vel_.a * position_r.y);
        target_vel_vec_r.y = platform_ramped_vel_.y + (platform_ramped_vel_.a * position_r.x);

        /* differential correction speed to minimise pivot_error */
        float delta_vel = pivot_error * wheel_param.pivot_kp;

        /* target velocity of left wheel (dot product with unit pivot vector) */
        float vel_l = target_vel_vec_l.x * unit_pivot_vector.x
                      + target_vel_vec_l.y * unit_pivot_vector.y;
        float target_vel_l = Utils::clip(vel_l - delta_vel,
                                                  wheel_param.max_linear_velocity,
                                                  -wheel_param.max_linear_velocity);

        /* target velocity of right wheel (dot product with unit pivot vector) */
        float vel_r = target_vel_vec_r.x * unit_pivot_vector.x
                      + target_vel_vec_r.y * unit_pivot_vector.y;
        float target_vel_r = Utils::clip(vel_r + delta_vel,
                                                  wheel_param.max_linear_velocity,
                                                  -wheel_param.max_linear_velocity);

        /* convert from linear to angular velocity */
        target_ang_vel_l = target_vel_l * wheel_param.linear_to_angular_velocity;
        target_ang_vel_r = target_vel_r * wheel_param.linear_to_angular_velocity;
    }

    std::ostream& operator << (std::ostream &out, const VelocityPlatformController& controller)
    {
        out << "num_of_wheels: " << controller.wheel_params_.size() << std::endl;

        for ( size_t i = 0; i < controller.wheel_params_.size(); i++ )
        {
            out << "wheel: " << i << std::endl;
            out << controller.wheel_params_[i] << std::endl;
            out << "------------------" << std::endl;
        }

        out << "target_vel: " << controller.platform_target_vel_ << std::endl;
        out << "ramped_vel: " << controller.platform_ramped_vel_ << std::endl;
       return out;
    }

} /* namespace kelo */
