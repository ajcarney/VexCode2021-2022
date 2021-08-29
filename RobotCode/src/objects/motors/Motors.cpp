/**
 * @file: ./RobotCode/src/motors/Motors.hpp
 * @author: Aiden Carney
 * @reviewed_on: 2/16/2020
 * @reviewed_by: Aiden Carney
 *
 * contains definition of global struct
 */

#include "Motors.hpp"
#include "MotorThread.hpp"

namespace Motors
{
    Motor back_right   {BR_MOTOR, CHASSIS_GEARSET, BR_REVERSED};
    Motor mid_right    {MR_MOTOR, CHASSIS_GEARSET, MR_REVERSED};
    Motor front_right  {FR_MOTOR, CHASSIS_GEARSET, FR_REVERSED};
    Motor back_left    {BL_MOTOR, CHASSIS_GEARSET, BL_REVERSED};
    Motor mid_left     {ML_MOTOR, CHASSIS_GEARSET, ML_REVERSED};
    Motor front_left   {FL_MOTOR, CHASSIS_GEARSET, FL_REVERSED};
    Motor mogo_lift    {MOGO_MOTOR, MOGO_GEARSET, MOGO_REVERSED};
    Motor lift         {LIFT_MOTOR, LIFT_GEARSET, LIFT_REVERSED};

    pros::ADIDigitalOut piston1 {PISTON1_MOTOR};
    pros::ADIDigitalOut piston2 {PISTON2_MOTOR};

    double chassis_gear_ratio = 3 / 5;

    std::array<Motor*, 8> motor_array = {
        &front_right,
        &front_left,
        &back_right,
        &back_left,
        &mid_right,
        &mid_left,
        &lift,
        &mogo_lift,
    };

    std::array<std::string, 8> motor_names_array = {
        "Front Left",
        "Front Right",
        "Back Left",
        "Back Right",
        "Left Intake",
        "Right Intake",
        "Upper Indexer",
        "Lower Indexer"
    };

    void enable_driver_control() {
        for(Motor* motor : motor_array) {
            motor->enable_driver_control();
        }
    }

    void disable_driver_control() {
        for(Motor* motor : motor_array) {
            motor->disable_driver_control();
        }
    }

    void set_brake_mode(pros::motor_brake_mode_e_t new_brakemode) {
        for(Motor* motor : motor_array) {
            motor->set_brake_mode(new_brakemode);
        }
    }

    void stop_all_motors() {
        for(Motor* motor : motor_array) {
            motor->move(0);
        }
    }

    void set_log_level(int log_level) {
        for(Motor* motor : motor_array) {
            motor->set_log_level(log_level);
        }
    }

    void register_motors() {
        MotorThread* motor_thread = MotorThread::get_instance();
        for(Motor* motor : motor_array) {
            motor_thread->register_motor(*motor);
        }
    }

    void unregister_motors() {
        MotorThread* motor_thread = MotorThread::get_instance();
        for(Motor* motor : motor_array) {
            motor_thread->unregister_motor(*motor);
        }
    }

};
