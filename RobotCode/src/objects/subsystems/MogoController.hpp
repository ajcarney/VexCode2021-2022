/**
 * @file: ./RobotCode/src/objects/subsystems/MogoController.hpp
 * @author: Aiden Carney
 * @reviewed_on:
 * @reviewed_by:
 *
 * Contains class for the mobile goal mogo subsystem
 */

#ifndef __MOGO_CONTROLLER_HPP__
#define __MOGO_CONTROLLER_HPP__

#include <tuple>
#include <queue>

#include "main.h"

#include "../motors/Motor.hpp"
#include "../sensors/Sensors.hpp"
#include "../../Configuration.hpp"


typedef enum e_mogo_command {
    e_mogo_start_up,
    e_mogo_start_down,
    e_mogo_move_to,
    e_mogo_stop
} mogo_command;

typedef struct {
    double setpoint=0;
    int max_velocity=150;
    int timeout=INT32_MAX;
    bool log_data=false;
    double motor_slew=INT32_MAX;
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
        
        std::vector<int> setpoints;

        static pid gains;

        int send_command(mogo_command command, mogo_args args={});

        static void mogo_motion_task(void*);

    public:
        MogoController(Motor& mogo);
        ~MogoController();

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
