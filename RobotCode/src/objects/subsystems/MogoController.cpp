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
    
    for(int i = 0; i<(sizeof(Configuration::mogo_setpoints) / sizeof(Configuration::mogo_setpoints[0])); i++) {
        setpoints.push_back(Configuration::mogo_setpoints[i]);
    }

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
            case e_mogo_start_up: {
                mogo_motor->set_voltage(12000);
                break;
            } case e_mogo_start_down: {
                mogo_motor->set_voltage(-12000);
                break;
            } case e_mogo_stop: {
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


/**
 * finds where the current index is in the list of setpoints
 * then increases or decreases the index to find the new setpoint
 * based on the parameter
 * then calls move_to to move to the new setpoint
 * caps the max and min and does not wrap back around
 */
int MogoController::cycle_setpoint(int direction, bool asynch) {
    int current_pot_value = mogo_motor->get_encoder_position();  // TODO: if you use a potentiometer set this to the pot value
    int target_set_point;

    std::vector<int> sorted_setpoints;
    for ( int i = 0; i < setpoints.size(); i++ ) {
        sorted_setpoints.push_back(setpoints[i] );
    }
    sorted_setpoints.push_back( current_pot_value );

    std::sort( sorted_setpoints.begin(), sorted_setpoints.end() );
    std::vector<int>::iterator elem = std::find (sorted_setpoints.begin(),
                                                 sorted_setpoints.end(),
                                                 current_pot_value);
    int index = std::distance(sorted_setpoints.begin(), elem);

    int loc = (std::abs(direction) / direction) + index;

    if ( !(loc < 0) || !(loc > setpoints.size() ))  //cap the max and min for the list
    {
        target_set_point = setpoints[loc];
        int uid = move_to(target_set_point, asynch);
        
        return uid;
    }
    
    return -INT32_MAX;
}


int MogoController::move_to(double sensor_value, bool asynch) {
    mogo_args args;
    args.end_state = sensor_value;
    int uid = send_command(e_mogo_move_to, args);
    if(!asynch) {
        wait_until_finished(uid);
    }
    return uid;
}


void MogoController::move_up() {
    send_command(e_mogo_start_up);
}

void MogoController::move_down() {
    send_command(e_mogo_start_down);
}


void MogoController::hard_stop() {
    reset_command_queue();
    send_command(e_mogo_stop);
}

void MogoController::stop() {
    send_command(e_mogo_stop);
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
