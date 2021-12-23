/**
 * @file: ./RobotCode/src/objects/subsystems/Indexer.cpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains implementation for the LiftController class
 */
#include <cassert>

#include "main.h"


#include "../serial/Logger.hpp"
#include "LiftController.hpp"

int LiftController::num_instances = 0;
std::queue<lift_action> LiftController::command_queue;
std::vector<int> LiftController::commands_finished;
std::atomic<bool> LiftController::command_start_lock = ATOMIC_VAR_INIT(false);
std::atomic<bool> LiftController::command_finish_lock = ATOMIC_VAR_INIT(false);

std::vector<Motor*> LiftController::motors;

pid LiftController::gains = {0.77, 0.000002, 7, INT32_MAX, INT32_MAX};


LiftController::LiftController(Motor &motor) {
    motors.clear();
    motors.push_back(&motor);

    for(Motor* m : motors) {
        m->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        m->set_motor_mode(e_voltage);
        m->disable_slew();
    }

    for(int i = 0; i<(sizeof(Configuration::lift_setpoints) / sizeof(Configuration::lift_setpoints[0])); i++) {
        setpoints.push_back(Configuration::lift_setpoints[i]);
    }

    if(num_instances == 0 || thread == NULL) {
        thread = new pros::Task( lift_motion_task, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift_thread");
    }

    num_instances += 1;
}


LiftController::LiftController(Motor& lift1, Motor& lift2) {
    motors.clear();
    motors.push_back(&lift1);
    motors.push_back(&lift2);

    for(Motor* m : motors) {
        m->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        m->set_motor_mode(e_voltage);
        m->disable_slew();
    }


    for(int i = 0; i<(sizeof(Configuration::lift_setpoints) / sizeof(Configuration::lift_setpoints[0])); i++) {
        setpoints.push_back(Configuration::lift_setpoints[i]);
    }

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
                for(Motor* m : motors) {
                    m->set_voltage(12000);
                }
                break;
            } case e_start_down: {
                for(Motor* m : motors) {
                    m->set_voltage(-12000);
                }
                break;
            } case e_stop: {
                for(Motor* m : motors) {
                    m->set_voltage(0);
                }
                break;
            } case e_move_to: {
                double prev_velocity = 0;
                double prev_error = 0;
                double integral = 0;

                std::vector<double> error_history;
                int max_history_length = 15;

                int current_time = pros::millis();
                int start_time = current_time;

                for(Motor* m : motors) {
                    m->set_motor_mode(e_builtin_velocity_pid);
                    m->disable_driver_control();
                }

                do {
                    int dt = pros::millis() - current_time;

                    long double error = action.args.setpoint - Sensors::lift_potentiometer.get_raw_value();

                    std::cout << error << "\n";

                  //  std::cout << prev_velocity << "\n";

                    integral = integral + (error * dt);
                    if(integral > gains.i_max) {
                        integral = gains.i_max;
                    } else if (integral < -gains.i_max) {
                        integral = -gains.i_max;
                    }

                    double derivative = error - prev_error;
                    prev_error = error;

                    current_time = pros::millis();

                    double abs_velocity = (gains.kP * error) + (gains.kI * integral) + (gains.kD * derivative);

                  //  std::cout << abs_velocity << "\n";

                    // slew rate code
                    double delta_velocity = abs_velocity - prev_velocity;
                    double slew_rate = action.args.motor_slew;
                    int over_slew = 0;
                    if(std::abs(delta_velocity) > (dt * slew_rate)) {
                        assert(delta_velocity != 0);

                        int sign = std::abs(delta_velocity) / delta_velocity;
                        std::cout << "over slew: " << sign << " " << dt << " " << slew_rate << "\n";
                        abs_velocity = prev_velocity + (sign * dt * slew_rate);
                        over_slew = 1;
                    }


                    // cap velocity to max velocity with regard to velocity
                    if ( std::abs(abs_velocity) > action.args.max_velocity ) {
                        abs_velocity = abs_velocity > 0 ? action.args.max_velocity : -action.args.max_velocity;
                    }
                    prev_velocity = abs_velocity;

                    error_history.push_back(prev_error);
                    if(error_history.size() > max_history_length) {
                         error_history.erase(error_history.begin());
                    }

                    double error_difference = *std::minmax_element(error_history.begin(), error_history.end()).second - *std::minmax_element(error_history.begin(), error_history.end()).first;

                    if ( action.args.log_data ) {
                        Logger logger;
                        log_entry entry;
                        entry.content = (
                            "[INFO] " + std::string("CHASSIS_PID_TURN")
                            + ", Time: " + std::to_string(pros::millis())
                            + ", Actual_Vol: " + std::to_string(motors.at(0)->get_actual_voltage())
                            + ", Brake: " + std::to_string(motors.at(0)->get_brake_mode())
                            + ", Gear: " + std::to_string(motors.at(0)->get_gearset())
                            + ", i_max: " + std::to_string(gains.i_max)
                            + ", I: " + std::to_string(integral)
                            + ", kD: " + std::to_string(gains.kD)
                            + ", kI: " + std::to_string(gains.kI)
                            + ", kP: " + std::to_string(gains.kP)
                            + ", Sp: " + std::to_string(action.args.setpoint)
                            + ", error history: " + std::to_string(error_history.size())
                            + ", history size: " + std::to_string(max_history_length)
                            + ", time out time: " + std::to_string(start_time + action.args.timeout)
                            + ", error difference: " + std::to_string(error_difference)
                            + ", over slew: " + std::to_string(over_slew)
                            + ", Actual_Vel: " + std::to_string(motors.at(0)->get_actual_velocity())
                        );
                        entry.stream = "clog";
                        logger.add(entry);
                    }

                    if (
                        std::abs(error_difference) < .007
                        && error_history.size() == max_history_length
                        && pros::millis() > start_time + 500  // don't stop right away on accident
                        // std::abs(l_difference) < 2
                        // && previous_l_velocities.size() == velocity_history
                        // && std::abs(r_difference) < 2
                        // && previous_r_velocities.size() == velocity_history
                        // && l_velocity < 2
                        // && r_velocity < 2
                    ) {  // velocity change has been minimal, so stop
                        for(Motor* m : motors) {
                            m->set_motor_mode(e_voltage);
                            m->set_voltage(0);
                        }
                        break; // end before timeout
                    }

                    for(Motor* m : motors) {
                        m->move_velocity(abs_velocity);
                        m->set_voltage(0);
                    }

                    std::cout << abs_velocity << "\n";

                    pros::delay(10);
                } while ( pros::millis() < (start_time + action.args.timeout) );

                for(Motor* m : motors) {
                    m->set_motor_mode(e_voltage);
                    m->set_voltage(0);
                    m->enable_driver_control();
                }

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
    action.uid = pros::millis() + motors.at(0)->get_actual_voltage();
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
int LiftController::cycle_setpoint(int direction, bool asynch) {
    int current_pot_value = Sensors::lift_potentiometer.get_raw_value();  // TODO: if you use a potentiometer set this to the pot value
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


int LiftController::move_to(double sensor_value, bool asynch, int timeout, int max_velocity, double motor_slew, bool log_data) {
    lift_action action;
    action.args.setpoint = sensor_value;
    action.args.max_velocity = max_velocity;
    action.args.timeout = timeout;
    action.args.log_data = log_data;
    action.args.motor_slew = motor_slew;
    int uid = send_command(e_move_to, action.args);
    if(!asynch) {
        wait_until_finished(uid);
    }
    return uid;
}



void LiftController::set_gains(pid new_gains) {
    gains.kP = new_gains.kP;
    gains.kI = new_gains.kI;
    gains.kD = new_gains.kD;
    gains.i_max = new_gains.i_max;
    gains.motor_slew = new_gains.motor_slew;
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
