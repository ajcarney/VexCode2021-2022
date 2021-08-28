/**
 * @file: ./RobotCode/src/motors/Motors.hpp
 * @author: Aiden Carney
 * @reviewed_on: 2/16/2020
 * @reviewed_by: Aiden Carney
 *
 * contains global struct for all motors
 */

#ifndef __MOTORS_HPP__
#define __MOTORS_HPP__

#include <array>

#include "main.h"

#include "../../Configuration.hpp"
#include "Motor.hpp"


namespace Motors
{
    extern Motor front_right;
    extern Motor front_left;
    extern Motor back_right;
    extern Motor back_left;
    extern Motor mid_right;
    extern Motor mid_left;
    extern Motor lift;
    extern Motor mogo_lift;

    extern pros::ADIDigitalOut piston1;
    extern pros::ADIDigitalOut piston2;

    extern double chassis_gear_ratio;

    extern std::array<Motor*, 8> motor_array;
    extern std::array<std::string, 8> motor_names_array;

    void enable_driver_control();
    void disable_driver_control();
    void set_brake_mode(pros::motor_brake_mode_e_t new_brakemode);
    void stop_all_motors();
    void set_log_level(int log_level);
    void register_motors();
    void unregister_motors();
};


#endif
