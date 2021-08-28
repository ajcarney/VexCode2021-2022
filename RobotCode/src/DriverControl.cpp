/**
 * @file: ./RobotCode/src/DriverControl.cpp
 * @author: Aiden Carney
 * @reviewed_on: 10/15/2019
 * @reviewed_by: Aiden Carney
 *
 * @see: DriverControl.hpp
 *
 */

#include <cstdlib>
#include <cmath>

#include "../include/main.h"

#include "objects/lcdCode/gui.hpp"
#include "objects/subsystems/chassis.hpp"
#include "objects/controller/controller.hpp"
#include "objects/motors/Motors.hpp"
#include "objects/sensors/Sensors.hpp"
#include "Configuration.hpp"
#include "DriverControl.hpp"
#include "Configuration.hpp"



/**
 * uses if statements to control motor based on controller settings
 */
void driver_control(void*)
{
    Autons autons;

    Controller controllers;

    Chassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Sensors::left_encoder, Sensors::right_encoder, CHASSIS_WIDTH, CHASSIS_GEAR_RATIO);

    int left_analog_y = 0;
    int right_analog_y = 0;

    while ( true ) {
        controllers.update_button_history();

    // section for chassis movement
        if(std::abs(controllers.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 5) {   // define deadzone for left analog input on the y axis
            left_analog_y = 0;
        } else {
            left_analog_y = controllers.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        }

        if(std::abs(controllers.master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 5) {   // define deadzone for right analog input on the y axis
            right_analog_y = 0;
        } else {
            right_analog_y = controllers.master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        }

        // float corrected_speed = ( .000043326431866017 * std::pow( leftDriveSpeed, 3 ) ) + ( 0.29594689028631 * leftDriveSpeed);  // cubic drive equation
        Motors::front_left.user_move(left_analog_y);
        Motors::mid_left.user_move(left_analog_y);
        Motors::back_left.user_move(left_analog_y);

        // float corrected_speed = ( .000043326431866017 * std::pow( rightDriveSpeed, 3 ) ) + ( 0.29594689028631 * rightDriveSpeed);  // cubic drive equation
        Motors::front_right.user_move(right_analog_y);
        Motors::mid_left.user_move(right_analog_y);
        Motors::back_right.user_move(right_analog_y);


        if ( controllers.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ) {
            autons.deploy();
        }

        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_UP) && controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_Y)) {
            autons.run_autonomous();
        }

        pros::delay(5);

    }
}
