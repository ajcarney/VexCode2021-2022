/**
 * @file: ./RobotCode/src/objects/subsystems/Indexer.cpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains implementation for the LiftController class
 */

#include "main.h"


#include "../serial/Logger.hpp"
#include "LiftController.hpp"

int LiftController::num_instances = 0;
std::queue<lift_action> LiftController::command_queue;
std::vector<int> LiftController::commands_finished;
std::atomic<bool> LiftController::command_start_lock = ATOMIC_VAR_INIT(false);
std::atomic<bool> LiftController::command_finish_lock = ATOMIC_VAR_INIT(false);

Motor* LiftController::lift_motor;


LiftController::LiftController(Motor &motor) {
    lift_motor = &motor;

    lift_motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    lift_motor->set_motor_mode(e_voltage);

    lift_motor->disable_slew();

    if(num_instances == 0 || thread == NULL) {
        thread = new pros::Task( lift_motion_task, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift_thread");
    }

    num_instances += 1;
}


LiftController::~LiftController() {
    num_instances -= 1;
    if(num_instances == 0) {
        delete thread;
    }
}



void LiftController::lift_motion_task(void*) {
    while(1) {
        while(1) { // delay unitl there is a command in the queue
            while ( command_start_lock.exchange( true ) ); //aquire lock and release it later if there is stuff in the queue
            if(!command_queue.empty()) {
                break;
            }

            command_start_lock.exchange( false ); //release lock
            pros::delay(5);
        }

        lift_action action = command_queue.front(); // lock is already owned
        command_queue.pop();
        command_start_lock.exchange( false ); //release lock

        // execute command
        switch(action.command) {
            case e_start_up: {
                lift_motor->set_voltage(12000);
                break;
            } case e_start_down: {
                lift_motor->set_voltage(-12000);
                break;
            } case e_stop: {
                lift_motor->set_voltage(0);
                break;
            }
        }

        while ( command_finish_lock.exchange( true ) ); //aquire lock
        commands_finished.push_back(action.uid);
        command_finish_lock.exchange( false ); //release lock
    }
}

int LiftController::send_command(lift_command command, lift_args args /*{}*/) {
    while ( command_start_lock.exchange( true ) ); //aquire lock
    lift_action action;
    action.command = command;
    action.args = args;
    action.uid = pros::millis() + lift_motor->get_actual_voltage();
    command_queue.push(action);
    command_start_lock.exchange( false ); //release lock

    return action.uid;
}

void LiftController::move_up() {
    send_command(e_start_up);
}

void LiftController::move_down() {
    send_command(e_start_down);
}


void LiftController::hard_stop() {
    reset_command_queue();
    send_command(e_stop);
}

void LiftController::stop() {
    send_command(e_stop);
}


void LiftController::reset_command_queue() {
    while ( command_start_lock.exchange( true ) ); //aquire lock
    std::queue<lift_action> empty_queue;
    std::swap( command_queue, empty_queue );  // replace command queue with an empty queue
    command_start_lock.exchange( false ); //release lock
}


void LiftController::wait_until_finished(int uid) {
    while(std::find(commands_finished.begin(), commands_finished.end(), uid) == commands_finished.end()) {
        pros::delay(10);
    }
    while ( command_finish_lock.exchange( true ) ); //aquire lock
    commands_finished.erase(std::remove(commands_finished.begin(), commands_finished.end(), uid), commands_finished.end());
    command_finish_lock.exchange( false ); //release lock
}


bool LiftController::is_finished(int uid) {
    if(std::find(commands_finished.begin(), commands_finished.end(), uid) == commands_finished.end()) {
        return false;  // command is not finished because it is not in the list
    }

    while ( command_finish_lock.exchange( true ) ); //aquire lock
    commands_finished.erase(std::remove(commands_finished.begin(), commands_finished.end(), uid), commands_finished.end());
    command_finish_lock.exchange( false ); //release lock

    return true;
}
