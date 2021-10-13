/**
 * @file: ./RobotCode/src/DriverControl.cpp
 * @author: Aiden Carney
 * @reviewed_on: 10/15/2019
 * @reviewed_by: Aiden Carney
 *
 * @see: DriverControl.hpp
 * see https://pros.cs.purdue.edu/v5/api/cpp/misc.html#pros-controller-digital-e-t
 * for button names
 *
 */

#include <cstdlib>
#include <cmath>

#include "../include/main.h"

#include "objects/lcdCode/gui.hpp"
#include "objects/subsystems/chassis.hpp"
#include "objects/subsystems/LiftController.hpp"
#include "objects/subsystems/MogoController.hpp"
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
    LiftController lift{Motors::lift};
    MogoController mogo{Motors::mogo_lift};

    int left_analog_y = 0;
    int right_analog_y = 0;

    bool p1_state = false;
    bool p2_state = false;
    Motors::piston1.set_value(false);
    Motors::piston2.set_value(false);



    const pros::controller_digital_e_t SHIFT_KEY = pros::E_CONTROLLER_DIGITAL_RIGHT; // TODO: set this to the actual shift key


    while ( true ) {
        controllers.update_button_history();

        if(std::abs(controllers.master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 5) {   // define deadzone for right analog input on the y axis
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
        Motors::mid_right.user_move(right_analog_y);
        Motors::back_right.user_move(right_analog_y);


    // piston movement
        if(controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {  // toggle piston 1
            p1_state = !p1_state;
            Motors::piston1.set_value(p1_state);
        }

        if(controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {  // toggle piston 2
            p2_state = !p2_state;
            Motors::piston2.set_value(p2_state);
        }

    // lift movement
        if(controllers.btn_is_pressing(SHIFT_KEY) && controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.cycle_setpoint(1, true);
        } else if(controllers.btn_is_pressing(SHIFT_KEY) && controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.cycle_setpoint(-1, true);
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move_up();
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move_down();
        } else {
            lift.stop();
        }

    // mogo movement
        if(controllers.btn_is_pressing(SHIFT_KEY) && controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            mogo.cycle_setpoint(1, true);
        } else if(controllers.btn_is_pressing(SHIFT_KEY) && controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            mogo.cycle_setpoint(-1, true);
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_L1)) {
            mogo.move_up();
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_L2)) {
            mogo.move_down();
        } else {
            mogo.stop();
        }

    // misc. functions
        if ( controllers.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ) {
            autons.deploy();
        }

        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_UP) && controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_Y)) {
            autons.run_autonomous();
        }

        pros::delay(5);

    }
}
