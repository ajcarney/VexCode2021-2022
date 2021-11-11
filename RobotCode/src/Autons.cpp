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
  Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift);
  MogoController mogo(Motors::mogo_lift);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});

  mogo.move_to(3500, false);
//  chassis.turn_left(51.5, 300, 1000, false); //Turn PID test
//  int uid = chassis.okapi_pid_straight_drive(1500, 10000, 6000, false, 0); //Drives foward to test pid_straight_drive
}




/**
 * skills autonomous
 */
void Autons::skills() {
  Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
  LiftController lift(Motors::lift);
  MogoController mogo(Motors::mogo_lift);
  PositionTracker* tracker = PositionTracker::get_instance();
  tracker->start_thread();
  tracker->enable_imu();
  tracker->set_log_level(0);
  tracker->set_position({0, 0, 0});
  chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
  chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});





    //Skills Auton
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    mogo.move_to(3500, false);
    int uid = chassis.okapi_pid_straight_drive(-600, 5000, 2000, false, 0);
    mogo.move_to(1700, false);
    Motors::piston3.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(400, 5000, 2000, false, 0);

    //PART 1
    pros::delay(2000);
    chassis.turn_right(85, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1900, 5000, 3000, false, 0);
    Motors::piston1.set_value(false);  //ADD PISTON OPEN
    lift.move_to(1000, true);
    chassis.turn_right(35, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1900, 5000, 3000, true, 0);
    lift.move_to(2200, true);
    chassis.wait_until_finished(uid);

    //Part 2
    pros::delay(2000);
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-100, 5000, 500, false, 0);
    chassis.turn_left(115, 300, 1000, false); //Turns right
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    mogo.move_to(3500, true);
    uid = chassis.okapi_pid_straight_drive(1500, 5000, 2000, true, 0);
    lift.move_to(900, true);
    chassis.wait_until_finished(uid);
    Motors::piston1.set_value(false);  //ADD PISTON OPEN

    //Part 3
    pros::delay(2000);
    uid = chassis.okapi_pid_straight_drive(-500, 5000, 1000, false, 0);
    chassis.turn_right(35, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(-2000, 5000, 2000, false, 0);
    uid = chassis.okapi_pid_straight_drive(-300, 2000, 1000, false, 0);
    uid = chassis.okapi_pid_straight_drive(-2000, 5000, 2000, false, 0);

    //Part 4
    pros::delay(2000);
    uid = chassis.okapi_pid_straight_drive(300, 5000, 1000, false, 0);
    chassis.turn_left(35, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1500, 5000, 1000, false, 0);
    lift.move_to(2200, false);
    chassis.turn_left(60, 300, 1000, false); //Turns right
    Motors::piston1.set_value(true);  //ADD PISTON OPEN

    //Part 5
    pros::delay(2000);
    chassis.turn_right(60, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(-2000, 5000, 2000, true, 0);
    lift.move_to(2200, true);
    chassis.wait_until_finished(uid);
    mogo.move_to(1700, false);
    Motors::piston3.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(700, 5000, 1000, false, 0);
    chassis.turn_right(85, 300, 1000, false); //Turns right

    //Part 6
    pros::delay(2000);
    uid = chassis.okapi_pid_straight_drive(1900, 5000, 1000, false, 0);
    Motors::piston1.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(1900, 5000, 1000, false, 0);
    chassis.turn_left(85, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1500, 5000, 1000, true, 0);
    lift.move_to(2200, true);
    chassis.wait_until_finished(uid);

    //Part 7
    pros::delay(2000);
    chassis.turn_right(50, 300, 1000, false); //Turns right
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    chassis.turn_left(50, 300, 1000, false); //Turns right
    lift.move_to(900, false);
    uid = chassis.okapi_pid_straight_drive(800, 5000, 1000, true, 0);

    //Part8
    pros::delay(2000);
    Motors::piston1.set_value(false);  //ADD PISTON OPEN
    lift.move_to(2200, false);
    chassis.turn_right(80, 300, 1000, false); //Turns right
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    chassis.turn_left(80, 300, 1000, false); //Turns right
    lift.move_to(900, false);

    //Part9
    pros::delay(2000);
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-3000, 5000, 1000, true, 0);
    mogo.move_to(1700, true);
    chassis.wait_until_finished(uid);
    uid = chassis.okapi_pid_straight_drive(1000, 5000, 1000, false, 0);
    chassis.turn_right(45, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1000, 5000, 1000, false, 0);
    Motors::piston1.set_value(false);  //ADD PISTON OPEN
    chassis.turn_left(90, 300, 1000, false); //Turns right
    uid = chassis.okapi_pid_straight_drive(1000, 5000, 1000, true, 0);
    lift.move_to(2200, true);
    chassis.wait_until_finished(uid);
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-500, 5000, 1000, false, 0);


}

    void Autons::win_point() {
      Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
      LiftController lift(Motors::lift);
      MogoController mogo(Motors::mogo_lift);
      PositionTracker* tracker = PositionTracker::get_instance();
      tracker->start_thread();
      tracker->enable_imu();
      tracker->set_log_level(0);
      tracker->set_position({0, 0, 0});
      chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
      chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});

  // Win Point
      lift.move_to(1350, false); //Moves lift up
      Motors::piston3.set_value(true);  //ADD PISTON OPEN
      int uid = chassis.okapi_pid_straight_drive(700, 5000, 500, false, 0); //drive forward a bit
      pros::delay(500);
      Motors::piston1.set_value(true); //Opens clamp to drop rings into mogo
      pros::delay(500);
      uid = chassis.okapi_pid_straight_drive(-800, 5000, 200, false, 0); //drive backwards a bit
      lift.move_to(950, false); //Moves lift down
      chassis.turn_right(85, 300, 1000, false); //Turns right
      uid = chassis.okapi_pid_straight_drive(-1300, 5000, 800, false, 0); //drive foward a bit
      chassis.turn_right(88, 300, 1000, false); //Turns right
      uid = chassis.okapi_pid_straight_drive(-3500, 5000, 3200, false, 0); //drive backwards a bit
      mogo.move_to(3500, false);
      uid = chassis.okapi_pid_straight_drive(-1000, 6000, 800, false, 0); //drive backwards a bit
      // mogo.move_to(50, false);
      // pros::delay(500);
      // lift.move_to(50, false);
      // pros::delay(500);
      // uid = chassis.okapi_pid_straight_drive(1000, 6000, 800, false, 0); //drive backwards a bit
}


//ADDED BY NOLAN PENDING REVIEW
void Autons::MidMogoLeft() {
  Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift);
    MogoController mogo(Motors::mogo_lift);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();
    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
    chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});


// Middle Mogo LEFT
  //  Motors::piston2.set_value(false);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston2.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    int uid = chassis.okapi_pid_straight_drive(-1900, 7000, 1650, false, 0); //Drives backwards into middle mogo
    Motors::piston2.set_value(false);   //ADD PISTON CLOSE
    uid = chassis.okapi_pid_straight_drive(1300, 7000, 1100, false, 0); //Drives back into home zone
    Motors::piston2.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(1000, 6000, 500, false, 0); //Drives back into home zone
    chassis.turn_left(68, 300, 1000, false); //Turns towards alliance mogo
    uid = chassis.okapi_pid_straight_drive(-300, 6000, 380, false, 0); //Drives into alliance mogo to drop rings
    lift.move_to(1350, false, 1500);  //ADD 6 BAR LIFT UP
    uid = chassis.okapi_pid_straight_drive(150, 6000, 380, false, 0); //Drives into alliance mogo to drop rings
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    pros::delay(250);
    uid = chassis.okapi_pid_straight_drive(-500, 6000, 1000, false, 0); //Reverses away from alliance mogo
    lift.move_to(900, false, 2000);  //ADD 6 BAR LIFT Down
    chassis.turn_left(175, 300, 1000, false); //Turns towards alliance mogo
    mogo.move_to(3500, false, 1000);
    Motors::piston2.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-1200, 4000, 1500, false, 0); //Reverses away from alliance mogo
    mogo.move_to(1700, false, 1500);
    Motors::piston3.set_value(false);  //ADD PISTON OPEN
}

void Autons::MidMogoRight() {
    Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift);
    MogoController mogo(Motors::mogo_lift);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();

    // R u using tabs instead of spaces!!!

    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
    chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});


    Motors::piston2.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    int uid = chassis.okapi_pid_straight_drive(-1825, 8000, 1500, false, 0); //Drives backwards into middle mogo
    Motors::piston2.set_value(false);    //ADD PISTON OPEN asynchronous WITH DRIVE
    uid = chassis.okapi_pid_straight_drive(1600, 7000, 800, false, 0); //Drives back into home zone
    Motors::piston2.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(1000, 6000, 300, false, 0); //Drives back into home zone
    Motors::piston2.set_value(false);
    chassis.turn_left(100, 300, 1000, false); //Turns towards alliance mogo
    lift.move_to(1300, false);
    uid = chassis.okapi_pid_straight_drive(900, 6000, 300, false, 0); //Drives back into home zone
    Motors::piston1.set_value(true);
    pros::delay(500);
    uid = chassis.okapi_pid_straight_drive(-500, 6000, 300, false, 0); //Drives back into home zone
    lift.move_to(950, false);
    chassis.turn_left(175, 300, 1000, false); //Turns towards alliance mogo
    mogo.move_to(3500, false, 1500);
    uid = chassis.okapi_pid_straight_drive(-1000, 4000, 1500, false, 0); //Drives back into home zone
    mogo.move_to(1700, false, 1500);
    Motors::piston3.set_value(false);
}

void Autons::CenterMogoleft() {
  Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift);
    MogoController mogo(Motors::mogo_lift);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();
    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
    chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});


// Middle Mogo LEFT
  //  Motors::piston2.set_value(false);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    Motors::piston1.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    int uid = chassis.okapi_pid_straight_drive(1500, 7000, 1650, false, 0); //Drives backwards into middle mogo
    chassis.turn_right(60, 300, 1000, false); //Turns towards alliance mogo
    uid = chassis.okapi_pid_straight_drive(1900, 7000, 1650, false, 0); //Drives backwards into middle mogo
    Motors::piston1.set_value(false);   //ADD PISTON CLOSE
    chassis.turn_left(15, 300, 1000, false); //Turns towards alliance mogo
    uid = chassis.okapi_pid_straight_drive(-2200, 7000, 3000, false, 0); //Drives back into home zone
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-300, 7000, 3000, false, 0); //Drives back into home zone
    chassis.turn_left(35, 300, 1000, false); //Turns towards alliance mogo
    mogo.move_to(3500, false);
    uid = chassis.okapi_pid_straight_drive(-2200, 7000, 3000, false, 0); //Drives back into home zone
    mogo.move_to(1700, false);
    Motors::piston3.set_value(false);  //ADD PISTON OPEN
    chassis.turn_left(90, 300, 1000, false); //Turns towards alliance mogo
    uid = chassis.okapi_pid_straight_drive(200, 7000, 3000, false, 0); //Drives back into home zone

}

void Autons::CenterMogoRight() {
  Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    LiftController lift(Motors::lift);
    MogoController mogo(Motors::mogo_lift);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();
    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});
    chassis.set_okapi_sdrive_gains({0.001, 0.0001, 0, INT32_MAX, INT32_MAX});


// Middle Mogo LEFT
  //  Motors::piston2.set_value(false);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston1.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    Motors::piston3.set_value(true);  //ADD PISTON OPEN
    chassis.turn_left(35, 300, 1000, false); //Turns towards alliance mogo
    int uid = chassis.okapi_pid_straight_drive(2800, 7000, 1650, false, 0); //Drives backwards into middle mogo
    Motors::piston1.set_value(false);   //ADD PISTON CLOSE
    uid = chassis.okapi_pid_straight_drive(-1900, 7000, 1100, false, 0); //Drives back into home zone
    Motors::piston1.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(-600, 6000, 500, false, 0); //Drives back into home zone
    chassis.turn_left(50, 300, 1000, false); //Turns towards alliance mogo
    mogo.move_to(3500, false, 1000);
    uid = chassis.okapi_pid_straight_drive(-1300, 6000, 2000, false, 0); //Drives into alliance mogo to drop rings
    mogo.move_to(1700, false, 2000);
    Motors::piston3.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(1000, 6000, 2000, false, 0); //Drives into alliance mogo to drop rings

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
