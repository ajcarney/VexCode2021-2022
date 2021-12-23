#ifndef __PTO_CHASSIS_HPP__
#define __PTO_CHASSIS_HPP__

#include <tuple>
#include <queue>

#include "main.h"

#include "../motors/Motor.hpp"
#include "../sensors/Sensors.hpp"
#include "../../Configuration.hpp"
#include "chassis.hpp"


typedef enum {
    e_pto_pid_straight_drive,
    e_pto_okapi_pid_straight_drive,
    e_pto_profiled_straight_drive,
    e_pto_turn,
    e_pto_drive_to_point,
    e_pto_turn_to_point,
    e_pto_turn_to_angle
} pto_chassis_commands;


typedef struct {
    double setpoint1=0;
    double setpoint2=0;
    int max_velocity=150;
    int max_voltage=11000;
    int max_heading_voltage_correction=5000;
    int timeout=INT32_MAX;
    int recalculations=0;
    int explicit_direction=0;
    double motor_slew=INT32_MAX;
    bool correct_heading=true;
    bool log_data=false;
} pto_chassis_params;


typedef struct {
    chassis_params args;
    int command_uid;
    chassis_commands command;
} pto_chassis_action;



class PTOChassis
{
    private:
        static Motor* r_back;
        static Motor* r_front;
        static Motor* r_extra;

        static Motor* l_back;
        static Motor* l_front;
        static Motor* l_extra;

        static pros::ADIDigitalOut* pto1;
        static pros::ADIDigitalOut* pto2;

        static Encoder* left_encoder;
        static Encoder* right_encoder;

        pros::Task *thread;  // the motor thread
        static std::queue<chassis_action> command_queue;
        static std::vector<int> commands_finished;
        static std::atomic<bool> command_start_lock;
        static std::atomic<bool> command_finish_lock;
        static int num_instances;

        static pid pid_sdrive_gains;
        static pid profiled_sdrive_gains;
        static pid okapi_sdrive_gains;
        static pid heading_gains;
        static pid turn_gains;

        static double get_angle_to_turn(double x, double y, int explicit_direction=1);
        static double get_angle_to_turn(double theta);

        static void t_pid_straight_drive(chassis_params args);  // functions called by thread for asynchronous movement
        static void t_okapi_pid_straight_drive(chassis_params args);
        static void t_profiled_straight_drive(chassis_params args);
        static void t_turn(chassis_params args);
        static void t_move_to_waypoint(chassis_params args, waypoint point);

        static double wheel_diameter;
        static double width;
        static double gear_ratio;

        static bool pto_state;

        static void allow_movement();
        static void stop_movement();

        static void chassis_motion_task(void*);


    public:
        PTOChassis(Motor &front_left, Motor &front_right, Motor &back_left, Motor &back_right, Motor &extra_left, Motor &extra_right, pros::ADIDigitalOut& piston1, pros::ADIDigitalOut& piston2, Encoder &l_encoder, Encoder &r_encoder, double chassis_width, double gearing=1, double wheel_size=3.25);
        ~PTOChassis();

        int pid_straight_drive(double encoder_ticks, int relative_heading=0, int max_velocity=450, int timeout=INT32_MAX, bool asynch=false, bool correct_heading=true, double slew=0.2, bool log_data=false);
        int profiled_straight_drive(double encoder_ticks, int max_velocity=450, int timeout=INT32_MAX, bool asynch=false, bool correct_heading=true, int relative_heading=0, bool log_data=false);
        int okapi_pid_straight_drive(double encoder_ticks, int max_voltage=11000, int timeout=INT32_MAX, bool asynch=false, int max_heading_correction=5000);
        int uneven_drive(double l_enc_ticks, double r_enc_ticks, int max_velocity=450, int timeout=INT32_MAX, bool asynch=false, double slew=10, bool log_data=false);
        int turn_right(double degrees, int max_velocity=450, int timeout=INT32_MAX, bool asynch=false, bool log_data=false);
        int turn_left(double degrees, int max_velocity=450, int timeout=INT32_MAX, bool asynch=false, bool log_data=false);
        int drive_to_point(double x, double y, int recalculations=0, int explicit_direction=0, int max_velocity=450, int timeout=INT32_MAX, bool correct_heading=true, bool asynch=false, double slew=10, bool log_data=false);
        int turn_to_point(double x, double y, int max_velocity=450, int timeout=INT32_MAX, bool asynch = false, double slew=10, bool log_data=false);
        int turn_to_angle(double theta, int max_velocity=450, int timeout=INT32_MAX, bool asynch = false, double slew=10, bool log_data=false);

        void set_pid_sdrive_gains(pid new_gains);
        void set_profiled_sdrive_gains(pid new_gains);
        void set_okapi_sdrive_gains(pid new_gains);
        void set_heading_gains(pid new_gains);
        void set_turn_gains(pid new_gains);



        static void pto_move_voltage(int r_voltage, int l_voltage);
        static void pto_move_velocity(int r_velocity, int l_velocity);
        void toggle_pto();
        void pto_enable_drive();
        void pto_enable_rings();

        /**
         * @param: int voltage -> the voltage on interval [-127, 127] to set the motor to
         * @return: None
         *
         * sets voltage of chassis
         */
        void move( int voltage );

        /**
         * @param: pros::motor_brake_mode_e_t new_brake_mode -> the new brakemode for the chassis
         * @return: None
         *
         * sets brake mode of all motors
         */
        void set_brake_mode( pros::motor_brake_mode_e_t new_brake_mode );





        /**
         * @return: None
         *
         * @see: Motors.hpp
         *
         * changes the direction at the api motor level so that all the
         * motors in the chassis system are reversed
         * useful for allowing to change direction of drive in user control
         */
        void change_direction();

        /**
         * @param: int speed -> the new speed the slew rate controller
         * @return: None
         *
         * sets the internal slew rate of the motor and enables it
         */
        void enable_slew( int rate=120 );

        /**
         * @return: None
         *
         * disables internal slew rate of the motor
         */
        void disable_slew( );






        void wait_until_finished(int uid);
        bool is_finished(int uid);


};


#endif
