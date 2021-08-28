/**
 * @file: ./RobotCode/src/objects/subsystems/Indexer.cpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains implementation for the MogoController class
 */

#include "main.h"


#include "../serial/Logger.hpp"
#include "MogoController.hpp"

int MogoController::num_instances = 0;
std::queue<mogo_action> MogoController::command_queue;
std::vector<int> MogoController::commands_finished;
std::atomic<bool> MogoController::command_start_lock = ATOMIC_VAR_INIT(false);
std::atomic<bool> MogoController::command_finish_lock = ATOMIC_VAR_INIT(false);

Motor* MogoController::mogo_motor;


MogoController::MogoController(Motor &motor) {
    mogo_motor = &motor;

    mogo_motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    mogo_motor->set_motor_mode(e_voltage);

    mogo_motor->disable_slew();

    if(num_instances == 0 || thread == NULL) {
        thread = new pros::Task( mogo_motion_task, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "mogo_thread");
    }

    num_instances += 1;
}


MogoController::~MogoController() {
    num_instances -= 1;
    if(num_instances == 0) {
        delete thread;
    }
}



void MogoController::mogo_motion_task(void*) {
    while(1) {
        while(1) { // delay unitl there is a command in the queue
            while ( command_start_lock.exchange( true ) ); //aquire lock and release it later if there is stuff in the queue
            if(!command_queue.empty()) {
                break;
            }

            command_start_lock.exchange( false ); //release lock
            pros::delay(5);
        }

        mogo_action action = command_queue.front(); // lock is already owned
        command_queue.pop();
        command_start_lock.exchange( false ); //release lock

        // execute command
        switch(action.command) {
            case e_start_up: {
                mogo_motor->set_voltage(12000);
                break;
            } case e_start_down: {
                mogo_motor->set_voltage(-12000);
                break;
            } case e_stop: {
                mogo_motor->set_voltage(0);
                break;
            }
        }

        while ( command_finish_lock.exchange( true ) ); //aquire lock
        commands_finished.push_back(action.uid);
        command_finish_lock.exchange( false ); //release lock
    }
}

int MogoController::send_command(mogo_command command, mogo_args args /*{}*/) {
    while ( command_start_lock.exchange( true ) ); //aquire lock
    mogo_action action;
    action.command = command;
    action.args = args;
    action.uid = pros::millis() + mogo_motor->get_actual_voltage();
    command_queue.push(action);
    command_start_lock.exchange( false ); //release lock

    return action.uid;
}

void MogoController::move_up() {
    send_command(e_start_up);
}

void MogoController::move_down() {
    send_command(e_start_down);
}


void MogoController::hard_stop() {
    reset_command_queue();
    send_command(e_stop);
}

void MogoController::stop() {
    send_command(e_stop);
}


void MogoController::reset_command_queue() {
    while ( command_start_lock.exchange( true ) ); //aquire lock
    std::queue<mogo_action> empty_queue;
    std::swap( command_queue, empty_queue );  // replace command queue with an empty queue
    command_start_lock.exchange( false ); //release lock
}


void MogoController::wait_until_finished(int uid) {
    while(std::find(commands_finished.begin(), commands_finished.end(), uid) == commands_finished.end()) {
        pros::delay(10);
    }
    while ( command_finish_lock.exchange( true ) ); //aquire lock
    commands_finished.erase(std::remove(commands_finished.begin(), commands_finished.end(), uid), commands_finished.end());
    command_finish_lock.exchange( false ); //release lock
}


bool MogoController::is_finished(int uid) {
    if(std::find(commands_finished.begin(), commands_finished.end(), uid) == commands_finished.end()) {
        return false;  // command is not finished because it is not in the list
    }

    while ( command_finish_lock.exchange( true ) ); //aquire lock
    commands_finished.erase(std::remove(commands_finished.begin(), commands_finished.end(), uid), commands_finished.end());
    command_finish_lock.exchange( false ); //release lock

    return true;
}
