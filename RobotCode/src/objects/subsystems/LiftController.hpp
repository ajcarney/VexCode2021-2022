/**
 * @file: ./RobotCode/src/objects/subsystems/LiftController.hpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains class for the mobile goal lift subsystem
 */

#ifndef __LIFT_CONTROLLER_HPP__
#define __LIFT_CONTROLLER_HPP__

#include <tuple>
#include <queue>

#include "main.h"

#include "../motors/Motor.hpp"
#include "../sensors/Sensors.hpp"
#include "../../Configuration.hpp"



typedef enum e_lift_command {
    e_start_up,
    e_start_down,
    e_move_to,
    e_stop
} lift_command;

typedef struct {
    double setpoint=0;
    int max_velocity=100;
    int timeout=INT32_MAX;
    bool log_data=false;
    double motor_slew=INT32_MAX;
}lift_args;

typedef struct {
    int uid;
    lift_command command;
    lift_args args;
} lift_action;

/**
 * @see: Motors.hpp
 *
 * contains methods to allow for control of the lift
 */
class LiftController
{
    private:
        static std::vector<Motor*> motors;

        static int num_instances;

        pros::Task *thread;  // the motor thread
        static std::queue<lift_action> command_queue;
        static std::vector<int> commands_finished;
        static std::atomic<bool> command_start_lock;
        static std::atomic<bool> command_finish_lock;

        std::vector<int> setpoints;

        static pid gains;

        int send_command(lift_command command, lift_args args={});

        static void lift_motion_task(void*);

    public:
        LiftController(Motor& motor);
        LiftController(Motor& lift1, Motor& lift2);
        ~LiftController();

        int cycle_setpoint(int direction, bool asynch);
        int move_to(double sensor_value, bool asynch, int timeout=INT32_MAX, int max_velocity=100, double motor_slew=INT32_MAX, bool log_data=false);

        void set_gains(pid new_gains);


        void move_down();
        void move_up();

        void hard_stop();
        void stop();

        void reset_command_queue();

        void wait_until_finished(int uid);
        bool is_finished(int uid);


};



#endif
