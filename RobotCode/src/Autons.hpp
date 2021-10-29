/**
 * @file: ./RobotCode/src/Autons.hpp
 * @author: Aiden Carney
 * @reviewed_on: 12/5/19
 * @reviewed_by: Aiden Carney
 *
 * contains class that holds data about the autonomous period as well as
 * structs for configuration data
 */

#ifndef __AUTONS_HPP__
#define __AUTONS_HPP__

#include <unordered_map>

#include "main.h"


typedef struct
{
    bool use_hardcoded = 0;
    bool gyro_turn = 1;
    bool accelleration_ctrl = 1;
    bool check_motor_tmp = 0;
    bool use_previous_macros = 1;
    bool record = 0;
} autonConfig;



/**
 * @see: Motors.hpp
 * @see: ./objects/lcdCode
 *
 * contains data for the autonomous period as well as functions to run the
 * selected autonomous
 */
class Autons
{
    private:

    public:
        Autons();
        ~Autons();

        static int selected_number;
        int debug_auton_num;       //change if more autons are added
                                   //debugger should be last option
        int driver_control_num;

        const std::unordered_map <int, const char*> AUTONOMOUS_NAMES = {
            {1, "Driver Control"},             //used to find name of auton
            {2, "pid_straight_drive"},                     //to keep title the same
            {3, "skills-current"},
            {4, "Win Point"},
            {5, "Middle Mogo Left"},
            {6, "Middle Mogo Right"},
            {7, "Debugger"},
        };
        const std::unordered_map <int, const char*> AUTONOMOUS_DESCRIPTIONS = {   //used to find color of auton
            {1, "goes directly to\ndriver control"},                               //selected to keep background the same
            {2, "test pid_straight_drive"},
            {3, "skills auton that scores ? points"},
            {4, "Gets auton win point"},
            {5, "Middle Mogo, set alliance mogo"},
            {6, "Middle Mogo, set alliance mogo"},
            {7, "opens debugger"},
        };
        const std::unordered_map <int, std::string> AUTONOMOUS_COLORS = {
            {1, "none"},                     //used to find color of auton
            {2, "none"},                     //selected to keep background the same
            {3, "none"},
            {4, "none"},
            {5, "none"},
            {6, "none"},
            {7, "none"},
        };

        void set_autonomous_number(int n);
        int get_autonomous_number();


        /**
         * @return: None
         *
         * @see: Motors.hpp
         * @see: Chassis.hpp
         *
         * get robot ready for auton
         */
        void deploy();

        /**
         * @return: None
         *
         * @see: Motors.hpp
         *
         * drives forward
         */
        void pid_straight_drive();

        /**
         * @return: None
         *
         * @see: Motors.hpp
         *
         * runs skills
         */
        void skills();

        /**
         * @return: None
         *
         * @see: Motors.hpp
         *
         * runs Middle Mogo Left
         */
        void win_point(); //ADDED BY NOLAN PENDING REVIEW

        void MidMogoLeft(); //ADDED BY NOLAN PENDING REVIEW

        void MidMogoRight();

        void run_autonomous();
};




#endif
