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
  chassis.set_turn_gains({2.9, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({1, 0, 0, INT32_MAX, INT32_MAX});

      chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
      chassis.okapi_pid_straight_drive(1000, 8000, 10000, false, 0); //Drives backwards into middle mogo


    // Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    // chassis.okapi_pid_straight_drive(1500, 12000, 950, false, 0);
    // Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    // chassis.okapi_pid_straight_drive(-1500, 12000, 1200, false, 0);





    // chassis.pto_enable_drive();
    // chassis.okapi_pid_straight_drive(1000, 7500, 10000, false, 0);
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
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1.5, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({.01, 0.000, 0, INT32_MAX, INT32_MAX});

    // //Skills Auton
    Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
    chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
    chassis.turn_right(7, 300, 2500, false); //Turn PID tes
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    chassis.okapi_pid_straight_drive(1700, 5000, 21000, false, 0);
    pros::delay(200);
    Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(50);
    lift.move_to(3400, false, 1000);
    chassis.turn_right(25, 300, 2500, false); //Turn PID tes
    pros::delay(100);
    chassis.okapi_pid_straight_drive(1600, 5000, 1950, false, 0);
    lift.move_to(4200, false, 1300);
    chassis.okapi_pid_straight_drive(900, 5000, 1100, false, 0);
    pros::delay(100);
    chassis.turn_left(31, 300, 2000, false); //Turn PID tes
    pros::delay(150);
    lift.move_to(3850, false, 1300);
    pros::delay(150);
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    pros::delay(100);
    lift.move_to(4000, false, 1300);
    pros::delay(100);
    chassis.okapi_pid_straight_drive(-500, 5000, 600, false, 0);
    lift.move_to(3050, false, 1300);
    pros::delay(250);
    chassis.turn_left(70, 300, 2000, false); //Turn PID tes
    pros::delay(50);
    // chassis.turn_left(39, 300, 2000, false); //Turn PID tes
    pros::delay(50);
    chassis.okapi_pid_straight_drive(1600, 5000, 2000, false, 0);
    Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(50);
    chassis.okapi_pid_straight_drive(-980, 5000, 1200, false, 0);
    pros::delay(150);
    lift.move_to(3400, false, 1000);
    pros::delay(150);
    chassis.turn_right(24, 300, 2000, false); //Turn PID tes
    pros::delay(50);
    Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(50);
    chassis.okapi_pid_straight_drive(-4900, 5000, 5800, false, 0);
    pros::delay(50);
    chassis.okapi_pid_straight_drive(400, 5000, 500, false, 0);
    pros::delay(50);
    lift.move_to(3400, false, 1000);
    pros::delay(150);
    chassis.turn_left(73, 300, 2000, false); //Turn PID tes
    pros::delay(100);
    lift.move_to(4200, false, 1000);
    chassis.okapi_pid_straight_drive(900, 5000, 1100, false, 0);
    pros::delay(100);
    chassis.turn_left(31, 300, 2000, false); //Turn PID tes
    pros::delay(150);
    lift.move_to(3850, false, 1300);
    pros::delay(150);
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    pros::delay(100);
    lift.move_to(4000, false, 1300);
    pros::delay(100);
    chassis.okapi_pid_straight_drive(-500, 5000, 600, false, 0);
    lift.move_to(3050, false, 1300);
    pros::delay(150);
    chassis.turn_left(40, 300, 2000, false); //Turn PID tes
    chassis.okapi_pid_straight_drive(-1000, 5000, 1200, false, 0);
    Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
    pros::delay(150);
    chassis.turn_right(40, 300, 2000, false); //Turn PID tes
    pros::delay(100);
    chassis.okapi_pid_straight_drive(1000, 5000, 1200, false, 0);

}

void Autons::win_point() {
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1.59, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({.01, 0.000, 0, INT32_MAX, INT32_MAX});

  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
  lift.move_to(3400, false, 1000);
  Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
  pros::delay(400);
  chassis.turn_left(52, 300, 1800, false); //Turn PID test
  chassis.okapi_pid_straight_drive(3650, 6000, 1100, false, 0);
  pros::delay(250);
  chassis.turn_left(107, 300, 2500, false); //Turn PID test
  chassis.okapi_pid_straight_drive(-2000, 6000, 2000, false, 0);
  Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
  chassis.okapi_pid_straight_drive(-1000, 6000, 1000, false, 0);
  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
  lift.move_to(3800, true, 800);
  int uid = chassis.okapi_pid_straight_drive(1500, 6000, 1500, true, 0);
  chassis.wait_until_finished(uid);
  chassis.pto_enable_rings();
  chassis.okapi_pid_straight_drive(200, 6000, 200, false, 0);
  chassis.start_run_rings(-12000);
  pros::delay(1500);
  chassis.stop_run_rings();
  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)

}


void Autons::MidMogoLeft() {
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1.55, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({.01, 0.000, 0, INT32_MAX, INT32_MAX});

  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
  Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
  chassis.okapi_pid_straight_drive(1500, 12000, 950, false, 0);
  Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
  chassis.okapi_pid_straight_drive(-1500, 12000, 1200, false, 0);
  chassis.okapi_pid_straight_drive(-900, 6000, 2000, false, 0);
  lift.move_to(4200, false, 1000);
  chassis.turn_left(78, 300, 2000, false);
  chassis.okapi_pid_straight_drive(100, 6000, 300, false, 0);
  Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  pros::delay(150);
  chassis.okapi_pid_straight_drive(-700, 6000, 2000, false, 0);
  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
  lift.move_to(4200, false, 1000);
  chassis.pto_enable_rings();  // make sure to call this function before running rings otherwise rings will not run
  chassis.okapi_pid_straight_drive(200, 7500, 200, false, 0);
  lift.move_to(4200, false, 1000);
  chassis.start_run_rings(-12000);
  chassis.okapi_pid_straight_drive(1100, 3000, 3000, false, 0);
  pros::delay(500);
  chassis.stop_run_rings();
  lift.move_to(4200, false, 1000);
  chassis.okapi_pid_straight_drive(-500, 3000, 1000, false, 0);

  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)


  // Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
  // pros::delay(250);
  // chassis.okapi_pid_straight_drive(-1000, 5000, 2000, false, 0);
  // chassis.turn_left(140, 300, 2000, false); //Turn PID test
  // chassis.okapi_pid_straight_drive(200, 5000, 200, false, 0);
  // Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
  // Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  // pros::delay(400);
  // chassis.okapi_pid_straight_drive(-650, 3000, 2000, false, 0);
  // Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  // Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
  // lift.move_to(3900, true, 1000);
  // chassis.pto_enable_rings();  // make sure to call this function before running rings otherwise rings will not run
  // chassis.okapi_pid_straight_drive(200, 7500, 200, false, 0);
  // chassis.start_run_rings(-12000);
  // chassis.okapi_pid_straight_drive(1100, 3000, 3000, false, 0);
  // pros::delay(500);
  // chassis.stop_run_rings();
  // Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  // Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  //
}

void Autons::MidMogoRight() {
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1.6, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({.01, 0.000, 0, INT32_MAX, INT32_MAX});

  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
  Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
  chassis.okapi_pid_straight_drive(1400, 7500, 1200, false, 0);
  Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
  lift.move_to(3800, true, 1000);
  int uid = chassis.okapi_pid_straight_drive(-1000, 7500, 900, true, 0);
  chassis.wait_until_finished(uid);
  lift.move_to(3000, true, 1000);
  pros::delay(750);
  Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
  pros::delay(250);
  chassis.okapi_pid_straight_drive(-200, 7500, 200, false, 0);
  chassis.turn_left(150, 300, 2000, false); //Turn PID test
  Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
  pros::delay(400);
  chassis.okapi_pid_straight_drive(-850, 7500, 800, false, 0);
  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
  chassis.turn_left(85, 300, 2000, false); //Turn PID test
  lift.move_to(3900, true, 1000);
  chassis.pto_enable_rings();  // make sure to call this function before running rings otherwise rings will not run
  chassis.okapi_pid_straight_drive(-200, 7500, 200, false, 0);
  chassis.start_run_rings(-12000);
  chassis.okapi_pid_straight_drive(1000, 3000, 3000, false, 0);
  pros::delay(500);
  chassis.stop_run_rings();
  Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
  Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)

}

void Autons::CenterMogoleft() {
  PTOChassis chassis(Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Motors::piston3, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift1, Motors::lift2);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({1.45, 0, 0, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({.01, 0.000, 0, INT32_MAX, INT32_MAX});

  // chassis.okapi_pid_straight_drive(-500, 5800, 600, false, 0); BASELINE

//Skills Version 2

//Part 1
    Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
    chassis.pto_enable_drive();  // make sure to call this function before running rings otherwise rings will not run
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    chassis.okapi_pid_straight_drive(-500, 5800, 600, false, 0);
    pros::delay(200);
    chassis.turn_left(26, 300, 2000, false); //Turn PID test
    pros::delay(25);
    chassis.okapi_pid_straight_drive(350, 5800, 420, false, 0);
    Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(25);
    lift.move_to(4200, false, 1000);
    pros::delay(25);
    chassis.okapi_pid_straight_drive(-500, 5800, 600, false, 0);
    pros::delay(25);

//Part 2
    lift.move_to(3400, false, 1000);
    pros::delay(250);
    chassis.turn_right(26, 300, 2000, false); //Turn PID test
    pros::delay(25);
    chassis.okapi_pid_straight_drive(-500, 5800, 600, false, 0);
    Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(150);
    chassis.okapi_pid_straight_drive(-700, 5800, 840, false, 0);
    pros::delay(75);
    Motors::piston2.set_value(false);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(true);  // do something to the claw (idk if this is open or close)
    pros::delay(50);
    chassis.okapi_pid_straight_drive(-1000, 5800, 1200, false, 0);
    pros::delay(25);

//Part 3
    lift.move_to(3400, false, 1000);
    pros::delay(250);
    chassis.turn_left(145, 300, 2000, false); //Turn PID test
    pros::delay(50);
    lift.move_to(4200, false, 1000);
    pros::delay(50);
    chassis.okapi_pid_straight_drive(700, 5800, 840, false, 0);
    pros::delay(50);
    lift.move_to(3950, false, 1000);
    chassis.okapi_pid_straight_drive(350, 5800, 420, false, 0);
    pros::delay(50);
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    pros::delay(100);
    lift.move_to(4200, false, 1000);
    pros::delay(50);
    chassis.okapi_pid_straight_drive(-400, 5800, 480, false, 0);

  //Part 4
    pros::delay(150);
    chassis.turn_right(43, 300, 2000, false); //Turn PID test
    pros::delay(150);
    chassis.okapi_pid_straight_drive(350, 5800, 420, false, 0);
    pros::delay(100);
    Motors::piston2.set_value(true);  // do something to the claw (idk if this is open or close)
    Motors::piston1.set_value(false);  // do something to the claw (idk if this is open or close)
    pros::delay(100);
    int uid = chassis.okapi_pid_straight_drive(1550, 5800, 1775, true, 0);
    lift.move_to(3100, true, 1000);
    chassis.wait_until_finished(uid);
    pros::delay(100);
    chassis.turn_left(79, 300, 2000, false); //Turn PID test

  //Part 5
    Motors::piston5.set_value(true);  // do something to the claw (idk if this is open or close)
    chassis.okapi_pid_straight_drive(400, 5800, 480, false, 0);
    Motors::piston5.set_value(false);  // do something to the claw (idk if this is open or close)
    lift.move_to(3200, false, 1000);
    chassis.okapi_pid_straight_drive(-4000, 5800, 4800, false, 0);
    pros::delay(25);

  //Part 6

  //Part 7


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
