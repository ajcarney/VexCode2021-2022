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
#include "objects/subsystems/pto_chassis.hpp"
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

    PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Motors::piston4, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift1, Motors::lift2);

    int left_analog_y = 0;
    int right_analog_y = 0;

    bool mogo_state = false;
    bool claw_state = false;
    Motors::piston1.set_value(false);
    Motors::piston2.set_value(false);
    Motors::piston5.set_value(false);
    Motors::piston6.set_value(false);

    chassis.pto_enable_drive();

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

        chassis.pto_user_move(right_analog_y, left_analog_y);  // use or don't use the extra motors based on if pto is enabled

    // pto shifting
        if(controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.pto_enable_rings();
        } else if(controllers.master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            chassis.pto_enable_drive();
        }


    // ring movement
        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_L1)) {
            chassis.start_run_rings(12000);
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_L2)) {
            chassis.start_run_rings(-12000);
        } else {
            chassis.stop_run_rings();
        }

    // lift movement
        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move_up();
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move_down();
        } else {
            lift.stop();
        }

    // mogo movement
        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_UP)) {
            Motors::piston1.set_value(true);
            Motors::piston2.set_value(true);

            mogo_state = true;
        } else if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            Motors::piston1.set_value(false);
            Motors::piston2.set_value(false);

            mogo_state = false;
        }

    // claw movement
        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if(claw_state) {
                Motors::piston5.set_value(false);
                Motors::piston6.set_value(false);
                claw_state = false;
            } else {
                Motors::piston5.set_value(true);
                Motors::piston6.set_value(true);
                claw_state = true;
            }
        }

    // misc. functions
        // if ( controllers.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ) {
        //     autons.deploy();
        // }

        if(controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_LEFT) && controllers.btn_is_pressing(pros::E_CONTROLLER_DIGITAL_A)) {
            autons.run_autonomous();
        }

        pros::delay(5);

    }
}
