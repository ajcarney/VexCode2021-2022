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



typedef enum e_lift_command {
    e_start_up,
    e_start_down,
    e_stop
} lift_command;

typedef struct {
    double end_state;
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
        static Motor *lift_motor;

        static int num_instances;

        pros::Task *thread;  // the motor thread
        static std::queue<lift_action> command_queue;
        static std::vector<int> commands_finished;
        static std::atomic<bool> command_start_lock;
        static std::atomic<bool> command_finish_lock;

        int send_command(lift_command command, lift_args args={});

        static void lift_motion_task(void*);

    public:
        LiftController(Motor& lift);
        ~LiftController();

        void move_down();
        void move_up();

        void hard_stop();
        void stop();

        void reset_command_queue();

        void wait_until_finished(int uid);
        bool is_finished(int uid);


};



#endif
