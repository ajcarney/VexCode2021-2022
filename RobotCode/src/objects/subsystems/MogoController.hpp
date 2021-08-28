/**
 * @file: ./RobotCode/src/objects/subsystems/MogoController.hpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains class for the mobile goal lift subsystem
 */

#ifndef __MOGO_CONTROLLER_HPP__
#define __MOGO_CONTROLLER_HPP__

#include <tuple>
#include <queue>

#include "main.h"

#include "../motors/Motor.hpp"
#include "../sensors/Sensors.hpp"



typedef enum e_mogo_command {
    e_start_up,
    e_start_down,
    e_stop
} mogo_command;

typedef struct {
    double end_state;
}mogo_args;

typedef struct {
    int uid;
    mogo_command command;
    mogo_args args;
} mogo_action;

/**
 * @see: Motors.hpp
 *
 * contains methods to allow for control of the mogo
 */
class MogoController
{
    private:
        static Motor *mogo_motor;

        static int num_instances;

        pros::Task *thread;  // the motor thread
        static std::queue<mogo_action> command_queue;
        static std::vector<int> commands_finished;
        static std::atomic<bool> command_start_lock;
        static std::atomic<bool> command_finish_lock;

        int send_command(mogo_command command, mogo_args args={});

        static void mogo_motion_task(void*);

    public:
        MogoController(Motor& mogo);
        ~MogoController();

        void move_down();
        void move_up();

        void hard_stop();
        void stop();

        void reset_command_queue();

        void wait_until_finished(int uid);
        bool is_finished(int uid);


};



#endif
