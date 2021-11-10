/**
 * @file: ./RobotCode/src/Configuration.hpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * contains class static variables for runtime and compile time configuration
 *
 * Set motor port, reversed, and gearset in this file
 * Set chassis width and gear ratio in this file (VERY IMPORTANT for position tracking and calculating the heading if not using imu)
 * Set sensor ports in this file
 *
 * WARNING:  IF YOU MODIFY THIS FILE ...
 *
 * BEWARE ... modifying this file can greatly increase your compile time (get it right the first time)
 *        you could try to fix this by going through each file in the project and deleting the line #include /.../Configuration.hpp
 *        somewhere near the top and seeing if the project still builds. I'm not going to do this because that is a boring and tedious process.
 *        You could also fix up the other include statements as well (if you get bored).
 *
 *        YOU WILL PROBABLY WANT TO DO THIS BEFORE TUNING PID LOOPS
 *
 * if you modify this file or any other *.hpp file you should run a prosv5 make all instead of
 * just the prosv5 make (doing this will occasionally help fix some wierd bugs)
 *
 * END WARNING
 */

#ifndef __CONFIGURATION_HPP__
#define __CONFIGURATION_HPP__

#include <iostream>
#include <vector>

#include "main.h"

#define BR_MOTOR                  19
#define MR_MOTOR                  20
#define FR_MOTOR                  10
#define BL_MOTOR                  14
#define ML_MOTOR                  15
#define FL_MOTOR                  2
#define MOGO_MOTOR                13
#define LIFT_MOTOR                1

#define BR_REVERSED               0
#define MR_REVERSED               1
#define FR_REVERSED               0
#define BL_REVERSED               1
#define ML_REVERSED               0
#define FL_REVERSED               1
#define MOGO_REVERSED             0
#define LIFT_REVERSED             1


#define PISTON1_MOTOR            'C'
#define PISTON2_MOTOR            'B'

// @nolan4s: see https://pros.cs.purdue.edu/v5/api/cpp/motors.html#pros-motor-gearset-e-t for setting it to different gear cartridges
#define CHASSIS_GEARSET           pros::E_MOTOR_GEARSET_06
#define LIFT_GEARSET              pros::E_MOTOR_GEARSET_36
#define MOGO_GEARSET              pros::E_MOTOR_GEARSET_36


#define LEFT_ENC_TOP_PORT        'G'
#define LEFT_ENC_BOTTOM_PORT     'H'
#define RIGHT_ENC_TOP_PORT       'F'
#define RIGHT_ENC_BOTTOM_PORT    'E'
#define STRAFE_ENC_TOP_PORT      'Z'
#define STRAFE_ENC_BOTTOM_PORT   'Z'
#define DETECTOR_MIDDLE_PORT     'Z'
#define LIFT_POTENTIOMETER_PORT  'H'
#define MOGO_POTENTIOMETER_PORT  'G'

#define DETECTOR_BOTTOM_PORT     'Z'
#define DETECTOR_TOP_PORT        'Z'

#define OPTICAL_PORT              12
#define DISTANCE_PORT             13
#define IMU_PORT                  9
#define EXPANDER_PORT             16

#define LED_R                     'Z'
#define LED_G                     'Z'
#define LED_B                     'Z'

#define CHASSIS_WIDTH            16
#define CHASSIS_GEAR_RATIO      5/3

#define LIFT_SETPOINTS  1, 2, 3
#define MOGO_SETPOINTS  1, 2, 3

typedef struct
{
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double i_max = 0;
    double motor_slew=INT32_MAX; //max rate of voltage change
    void print() {
        std::cout << "kP: " << this->kP << "\n";
        std::cout << "kI: " << this->kI << "\n";
        std::cout << "kD: " << this->kD << "\n";
        std::cout << "I_max: " << this->i_max << "\n";
        std::cout << "motor_slew: " << this->motor_slew << "\n";
    };
} pid;


class Configuration {
    public:
        static constexpr pid internal_motor_pid {1, 0, 0, 0, INT32_MAX};  // see above struct for parameter order
        static constexpr pid lift_pid {1, 0, 0, 0, INT32_MAX};
        static constexpr pid chassis_pid {1, 0, 0, 0, INT32_MAX};

        static constexpr int lift_setpoints[] = {};
        static constexpr int mogo_setpoints[] = {};
};

#endif
