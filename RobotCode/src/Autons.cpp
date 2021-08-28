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
#include "objects/lcdCode/DriverControl/AutonomousLCD.hpp"


int Autons::selected_number = 1;

Autons::Autons( )
{
    debug_auton_num = 4;  // TODO: this should be dynamically set because it causes a lot of errors otherwise
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
void Autons::one_pt() {
    deploy();
}




/**
 * skills autonomous
 */
void Autons::skills() {
    Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->start_thread();
    tracker->enable_imu();
    tracker->set_log_level(0);
    tracker->set_position({0, 0, 0});
    chassis.set_turn_gains({4, 0.0001, 20, INT32_MAX, INT32_MAX});


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

}



void Autons::run_autonomous() {
    switch(selected_number) {
        case 1:
           break;

        case 2:
            one_pt();
            break;

        case 3:
            skills();
            break;
    }
}
