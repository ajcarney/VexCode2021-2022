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
    debug_auton_num = 7;  // TODO: this should be dynamically set because it causes a lot of errors otherwise  //ADDED BY NOLAN PENDING REVIEW
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


//  chassis.turn_left(51.5, 300, 1000, false); //Turn PID test
  int uid = chassis.okapi_pid_straight_drive(1500, 10000, 6000, false, 0); //Drives foward to test pid_straight_drive
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
        int uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives fowards into middle mogo
  //      lift.move_to(50, false); //Move lift up
        Motors::piston1.set_value(false);   //ADD PISTON OPEN to drop rings into mogo
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives back
    //    lift.move_to(-50, false); //ADD SIX BAR LIFT Down
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into middle goal
        Motors::piston1.set_value(true);   //ADD PISTON OPEN to drip rings into
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives back into home zone
        chassis.turn_left(10, 300, 1000, false); //Turns away towards alliance mogo
      //  mogo.move_to(-50,false);   //ADD MOGO HOLDER DOWN
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Backs into alliance mogo
      //  mogo.move_to(50,false); //Lifts alliance mogo
        chassis.turn_right(10, 300, 1000, false); //Turns towards bridge
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Backs away from bridge a bit
      //  lift.move_to(50, false); //ADD 6 BAR LIFT Up
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into bridge
        Motors::piston1.set_value(false);   //ADD PISTON OPEN (scores middle mogo on bridge)
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Reverses away from bridge
        //lift.move_to(-50, false);   //ADD 6 BAR LIFT Down
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into alliance mogo
        chassis.turn_right(10, 300, 1000, false); //Turns towards middle center mogo
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into middle center mogo
        Motors::piston1.set_value(true);   //ADD PISTON CLOSE
        chassis.turn_right(10, 300, 1000, false); //Turns towards bridge
      //  lift.move_to(50, false);    //ADD SIX BAR UP
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into bridge
    //    lift.move_to(-50, false); //ADD SIX BAR DOWN (Just a litte)
        Motors::piston1.set_value(false); //ADD PISTON open
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Backs up slightly
      //  lift.move_to(50, false);//ADD SIX BAR UP (Just a little)
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Backs Away from bridge
    //    lift.move_to(-50, false);//ADD SIX BAR DOWN
        chassis.turn_right(10, 300, 1000, false); //Turns towards second alliance mogo
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into second alliance mogo
        Motors::piston1.set_value(true);   //ADD PISTON CLOSE
        chassis.turn_right(10, 300, 1000, false); //Turns towards other home zone
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives into other home zone
        Motors::piston1.set_value(false);   //ADD PISTON open
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Reverse a bit
        chassis.turn_right(10, 300, 1000, false); //Turns 90 degrees left
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Reverse a bit
      //  mogo.move_to(-50,false); //ADD MOGO HOLDER DOWN
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives foward releasing alliance mogo
        chassis.turn_right(10, 300, 1000, false); //Turns 90 degrees left
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives foward into final center mogo
        Motors::piston1.set_value(true);  //ADD PISTON CLOSE
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives foward into home zone
        Motors::piston1.set_value(false);   //ADD PISTON open
        uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, false, 0); //Drives backwards a bit

}



    // // tower 1
    // int uid = chassis.okapi_pid_straight_drive(-770, 6000, 2500, true, 0);
    // indexer.run_upper_roller();
    // while(!chassis.is_finished(uid)) {
    //     pros::delay(5);
    // }
    //
    // uid = chassis.turn_right(43, 300, 2500, true);
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     pros::delay(5);
    // }
    // indexer.stop();
    //
    // uid = chassis.okapi_pid_straight_drive(1100, 6000, 3000, true, 2200);   // pick up next ball
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     pros::delay(5);
    // }
    // indexer.stop();
    //
    // // score first tower
    // indexer.index();  // score first two
    // pros::delay(700);
    // indexer.stop();
    //
    // uid = chassis.okapi_pid_straight_drive(-1200, 6000, 2000, false, 0);
    //
    // chassis.turn_to_angle(93, 300, 3000, false);
    // chassis.okapi_pid_straight_drive(-6000, 6000, 10000, false, 1000);
    // chassis.okapi_pid_straight_drive(700, 6000, 2000, false, 1000);
    //
    // chassis.turn_right(93, 200, 4000, false);
    //
    // uid = chassis.okapi_pid_straight_drive(3500, 5000, 6500, true, 0);   // pick up next ball
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     intakes.intake();
    //     pros::delay(5);
    // }
    // indexer.stop();
    // intakes.stop();
    //
    // uid = chassis.turn_right(45, 300, 2500, true);
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     pros::delay(5);
    // }
    // indexer.stop();
    //
    // uid = chassis.okapi_pid_straight_drive(1000, 6000, 3000, true, 2200);   // pick up next ball
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     pros::delay(5);
    // }
    // indexer.stop();
    //
    // indexer.index();  // score first two
    // pros::delay(700);
    // indexer.stop();
    //
    //
    // uid = chassis.okapi_pid_straight_drive(-700, 6000, 2000, false, 0);
    //
    // chassis.turn_left(42, 300, 2500, false);
    // chassis.okapi_pid_straight_drive(-1050, 6000, 3000, false, 0);
    // chassis.turn_left(90, 300, 2500, false);
    // chassis.okapi_pid_straight_drive(-2000, 6000, 3000, false, 0);
    //
    // uid = chassis.okapi_pid_straight_drive(2800, 5000, 7000, true, 0);   // pick up next ball
    // while(!chassis.is_finished(uid)) {
    //     indexer.increment();
    //     intakes.intake();
    //     pros::delay(5);
    // }
    // indexer.stop();
    // intakes.stop();
    // for(int i = 0; i < 100; i++) {
    //     indexer.increment();
    //     intakes.intake();
    //     pros::delay(10);
    // }
    // indexer.stop();
    // intakes.stop();
    //
    // chassis.turn_left(90, 300, 2500, false);
    // intakes.rocket_outward();
    // chassis.okapi_pid_straight_drive(1000, 4000, 5000, false, 0);
    //
    // indexer.index_no_backboard();  // score first two
    // pros::delay(700);
    // indexer.stop();
    //
    // chassis.okapi_pid_straight_drive(-500, 4000, 4000, false, 0);
    // chassis.turn_left(10, 300, 1000, false);
    // intakes.intake();
    // pros::delay(500);
    // chassis.move(127);
    // pros::delay(1000);
    // chassis.move(0);
    // intakes.stop();
    // chassis.okapi_pid_straight_drive(-500, 4000, 4000, false, 0);
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
      lift.move_to(50, false); //Moves lift up
      int uid = chassis.okapi_pid_straight_drive(400, 6000, 200, true, 0); //drive forward a bit
      Motors::piston1.set_value(false); //Opens clamp to drop rings into mogo
      uid = chassis.okapi_pid_straight_drive(-400, 6000, 200, true, 0); //drive backwards a bit
      lift.move_to(-50, true); //Moves lift down
      chassis.wait_until_finished(uid); //waits for previous statements to finish
      chassis.turn_right(51.5, 300, 1000, false); //Turns right
      uid = chassis.okapi_pid_straight_drive(-1000, 6000, 800, false, 0); //drive foward a bit
      chassis.turn_right(51.5, 300, 1000, false); //Turns right
      uid = chassis.okapi_pid_straight_drive(-3500, 6000, 3200, false, 0); //drive backwards a bit
      mogo.move_to(-50, false);
      uid = chassis.okapi_pid_straight_drive(-1000, 6000, 800, false, 0); //drive backwards a bit
      mogo.move_to(50, false);
      pros::delay(500);
      lift.move_to(50, false);
      pros::delay(500);
      uid = chassis.okapi_pid_straight_drive(1000, 6000, 800, false, 0); //drive backwards a bit
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
    Motors::piston1.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    int uid = chassis.okapi_pid_straight_drive(-2200, 6000, 2000, true, 0); //Drives backwards into middle mogo
    Motors::piston2.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    chassis.wait_until_finished(uid);
    Motors::piston2.set_value(false);   //ADD PISTON CLOSE
    uid = chassis.okapi_pid_straight_drive(1500, 6000, 1300, false, 0); //Drives back into home zone
    Motors::piston2.set_value(false);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(700, 6000, 500, false, 0); //Drives back into home zone
    chassis.turn_left(60, 300, 1000, false); //Turns towards alliance mogo
    Motors::piston2.set_value(false);  //ADD PISTON CLOSE
    pros::delay(500); //let turn settle
    lift.move_to(50, false);  //ADD 6 BAR LIFT UP
    uid = chassis.okapi_pid_straight_drive(600, 6000, 380, false, 0); //Drives into alliance mogo to drop rings
    Motors::piston1.set_value(false);  //ADD PISTON OPEN
    pros::delay(500);
    uid = chassis.okapi_pid_straight_drive(-600, 6000, 380, false, 0); //Reverses away from alliance mogo
    lift.move_to(-50, false);  //ADD 6 BAR LIFT Down
    uid = chassis.okapi_pid_straight_drive(625, 6000, 2500, false, 0); //Drives into alliance mogo
    Motors::piston1.set_value(true);  //ADD PISTON CLOSE
    lift.move_to(50, false);  //ADD 6 BAR LIFT UP
    uid = chassis.okapi_pid_straight_drive(150, 6000, 2500, false, 0); //Backs away from bridge in sync with the six bar lift
    Motors::piston1.set_value(false);
    uid = chassis.okapi_pid_straight_drive(-300, 6000, 2500, false, 0); //Backs away from bridge in sync with the six bar lift
    lift.move_to(50, false);  //ADD 6 BAR LIFT Down

}

void Autons::MidMogoRight() {
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


    Motors::piston2.set_value(true);    //ADD PISTON OPEN asynchronous WITH DRIVE
    int uid = chassis.okapi_pid_straight_drive(-1850, 7000, 1500, false, 0); //Drives backwards into middle mogo
//    pros::delay(500);
    Motors::piston2.set_value(false);    //ADD PISTON OPEN asynchronous WITH DRIVE
    uid = chassis.okapi_pid_straight_drive(1600, 7000, 800, false, 0); //Drives back into home zone
    Motors::piston2.set_value(true);  //ADD PISTON OPEN
    uid = chassis.okapi_pid_straight_drive(1000, 6000, 300, false, 0); //Drives back into home zone
    Motors::piston2.set_value(false);
    chassis.turn_right(72, 300, 1000, false); //Turns towards alliance mogo
    mogo.move_to(-2000, false);
    uid = chassis.okapi_pid_straight_drive(-800, 4000, 1000, false, 0); //Drives back into home zone
  //  mogo.move_to(50, false);
    pros::delay(500);
//    lift.move_to(50, false);
    pros::delay(500);
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
    }
}
