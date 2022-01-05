/**
 * @file: ./RobotCode/src/Autons.cpp
 * @author: Aiden Carney
 * @reviewed_on: 12/5/19
 * @reviewed_by: Aiden Carney
 *
 * @see: Autons.hpp
 *
 * contains implementation for autonomous options
 */

#include <unordered_map>

#include "main.h"

#include "Autons.hpp"
#include "objects/motors/Motors.hpp"
#include "objects/motors/MotorThread.hpp"
#include "objects/position_tracking/PositionTracker.hpp"
#include "objects/subsystems/chassis.hpp"
#include "objects/subsystems/pto_chassis.hpp"
#include "objects/subsystems/MogoController.hpp"
#include "objects/subsystems/LiftController.hpp"
#include "objects/lcdCode/DriverControl/AutonomousLCD.hpp"


int Autons::selected_number = 1;

Autons::Autons( )
{
    debug_auton_num = 9;  // TODO: this should be dynamically set because it causes a lot of errors otherwise  //ADDED BY NOLAN PENDING REVIEW
    driver_control_num = 1;
}



Autons::~Autons( ) {

}

void Autons::set_autonomous_number(int n) {
    selected_number = n;
}

int Autons::get_autonomous_number() {
    return selected_number;
}

/**
 * deploys by outtaking and running top roller
 */
void Autons::deploy() {

}


/**
 * drives forward to score in the zone, then drive backward
 * to stop touching the cube
 */


void Autons::pid_straight_drive() {
    PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift1, Motors::lift2);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();
    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({1, 0.001, 20, INT32_MAX, INT32_MAX});
    chassis.set_okapi_sdrive_gains({1, 0.0001, 0, INT32_MAX, INT32_MAX});

    // chassis.pto_enable_drive();
    chassis.okapi_pid_straight_drive(1900, 7500, 1000, false, 0);
    // pros::delay(2000);
    // chassis.turn_right(90, 300, 2000, false); //Turn PID test
    // pros::delay(2000);
    // chassis.turn_left(90, 300, 2000, false); //Turn PID test
    // pros::delay(2000);

    //
    // chassis.pto_enable_rings();  // make sure to call this function before running rings otherwise rings will not run
    // chassis.start_run_rings();
    // pros::delay(1000);
    // chassis.stop_run_rings();
    //
    // chassis.set_okapi_sdrive_gains({0.002, 0.0001, 0, INT32_MAX, INT32_MAX});  // change pid constants because driving backwards with less motors
    // chassis.okapi_pid_straight_drive(-1000);  // this drives backwards without the extra motors because the enable drive function was not called
    //
    // chassis.pto_enable_drive();  // enable the extra motors again
    // chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});  // change pid constants back to default because 6 motors are enabled again
    //
    //
    // Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    // Motors::piston6.set_value(false);
    //
    // Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
    // Motors::piston2.set_value(true);  // do something to the mogo (idk if this is open or close)
    //
//  chassis.turn_left(51.5, 300, 1000, false); //Turn PID test
//  int uid = chassis.okapi_pid_straight_drive(1500, 10000, 6000, false, 0); //Drives foward to test pid_straight_drive
}




/**
 * skills autonomous
 */
void Autons::skills() {
  // Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  // LiftController lift(Motors::lift);
  // MogoController mogo(Motors::mogo_lift);
  // PositionTracker* tracker = PositionTracker::get_instance();
  // tracker->start_thread();
  // tracker->enable_imu();
  // tracker->set_log_level(0);
  // tracker->set_position({0, 0, 0});
  // chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
  // chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});
    //
    //
    //
    //
    //
    // //Skills Auton

}

void Autons::win_point() {
  //     Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  //     LiftController lift(Motors::lift);
  //     MogoController mogo(Motors::mogo_lift);
  //     PositionTracker* tracker = PositionTracker::get_instance();
  //     tracker->start_thread();
  //     tracker->enable_imu();
  //     tracker->set_log_level(0);
  //     tracker->set_position({0, 0, 0});
  //     chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
  //     chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});
  //
  // // Win Point

}


void Autons::MidMogoLeft() {
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1, 0.001, 20, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({1, 0.0001, 0, INT32_MAX, INT32_MAX});

    // chassis.pto_enable_drive();
    chassis.okapi_pid_straight_drive(1900, 7500, 1000, false, 0);

    //
    // chassis.pto_enable_rings();  // make sure to call this function before running rings otherwise rings will not run
    // chassis.start_run_rings();
    // pros::delay(1000);
    // chassis.stop_run_rings();
    //
    // chassis.set_okapi_sdrive_gains({0.002, 0.0001, 0, INT32_MAX, INT32_MAX});  // change pid constants because driving backwards with less motors
    // chassis.okapi_pid_straight_drive(-1000);  // this drives backwards without the extra motors because the enable drive function was not called
    //
    // chassis.pto_enable_drive();  // enable the extra motors again
    // chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});  // change pid constants back to default because 6 motors are enabled again
    //
    //
    // Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    // Motors::piston6.set_value(false);
    //
    // Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
    // Motors::piston2.set_value(true);  // do something to the mogo (idk if this is open or close)
    //
//  chassis.turn_left(51.5, 300, 1000, false); //Turn PID test
//  int uid = chassis.okapi_pid_straight_drive(1500, 10000, 6000, false, 0); //Drives foward to test pid_straight_drive
}

void Autons::MidMogoRight() {
  //   Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  //     LiftController lift(Motors::lift);
  //     MogoController mogo(Motors::mogo_lift);
  //     PositionTracker* tracker = PositionTracker::get_instance();
  //     tracker->start_thread();
  //     tracker->enable_imu();
  //     tracker->set_log_level(0);
  //     tracker->set_position({0, 0, 0});
  //     chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
  //     chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});

}

void Autons::CenterMogoleft() {
//   Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
//     LiftController lift(Motors::lift);
//     MogoController mogo(Motors::mogo_lift);
//     PositionTracker* tracker = PositionTracker::get_instance();
//     tracker->start_thread();
//     tracker->enable_imu();
//     tracker->set_log_level(0);
//     tracker->set_position({0, 0, 0});
//     chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
//     chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});
//
//
// // Middle Mogo LEFT

}

void Autons::CenterMogoRight() {
//   Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
//     LiftController lift(Motors::lift);
//     MogoController mogo(Motors::mogo_lift);
//     PositionTracker* tracker = PositionTracker::get_instance();
//     tracker->start_thread();
//     tracker->enable_imu();
//     tracker->set_log_level(0);
//     tracker->set_position({0, 0, 0});
//     chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
//     chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});
//
//
// // Middle Mogo LEFT

}


void Autons::run_autonomous() {
    switch(selected_number) {
        case 1:
           break;

        case 2:
            pid_straight_drive();
            break;

        case 3:
            skills();
            break;

        case 4:
            win_point();
            break;

        case 5:
            MidMogoLeft();
            break;

        case 6:
            MidMogoRight();
            break;

        case 7:
            CenterMogoleft();
            break;

        case 8:
            CenterMogoRight();
            break;
    }
}
