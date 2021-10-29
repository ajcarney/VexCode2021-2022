#include <csignal>

#include "main.h"

#include "Autons.hpp"
#include "Configuration.hpp"
#include "DriverControl.hpp"
#include "objects/controller/controller.hpp"
#include "objects/lcdCode/DriverControl/AutonomousLCD.hpp"
#include "objects/lcdCode/DriverControl/DriverControlLCD.hpp"
#include "objects/lcdCode/gui.hpp"
#include "objects/lcdCode/TemporaryScreen.hpp"
#include "objects/motors/Motors.hpp"
#include "objects/motors/MotorThread.hpp"
#include "objects/position_tracking/PositionTracker.hpp"
#include "objects/serial/Logger.hpp"
#include "objects/serial/Server.hpp"
#include "objects/subsystems/chassis.hpp"
#include "objects/sensors/RGBLed.hpp"



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 void initialize()
 {
    pros::c::serctl(SERCTL_ACTIVATE, 0);  // I think this enables stdin (necessary to start server)

    Motors::register_motors();
    MotorThread::get_instance()->start_thread();

    pros::delay(100); //wait for terminal to start and lvgl

    int final_auton_choice = chooseAuton();
    Autons auton;
    auton.set_autonomous_number(final_auton_choice);

     Sensors::calibrate_imu();  // TODO: uncomment when you have an imu plugged in


    // std::cout << OptionsScreen::cnfg.use_hardcoded << '\n';
    // std::cout << OptionsScreen::cnfg.gyro_turn << '\n';
    // std::cout << OptionsScreen::cnfg.accelleration_ctrl << '\n';
    // std::cout << OptionsScreen::cnfg.check_motor_tmp << '\n';
    // std::cout << OptionsScreen::cnfg.use_previous_macros << '\n';
    // std::cout << OptionsScreen::cnfg.record << '\n';

    std::cout << "initalize finished" << "\n";
    lv_scr_load(tempScreen::temp_screen);
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    AutonomousLCD auton_lcd;
    Autons auton;
    auton_lcd.update_labels(auton.get_autonomous_number());
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    Autons auton;
    auton.run_autonomous();
}


 void log_thread_fn( void* )
 {
     Logger logger;
     Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);

     double kP = Configuration::chassis_pid.kP;
     double kI = Configuration::chassis_pid.kI;
     double kD = Configuration::chassis_pid.kD;
     double I_max = Configuration::chassis_pid.i_max;

     int l_id = Sensors::left_encoder.get_unique_id();
     int r_id = Sensors::right_encoder.get_unique_id();
     // double prev_l_encoder = std::get<0>(chassis.get_average_encoders(l_id, r_id));
     // double prev_r_encoder = std::get<1>(chassis.get_average_encoders(l_id, r_id));
     // double intitial_angle = Sensors::imu.get_heading();
     // double prev_angle = Sensors::imu.get_heading();
     // double relative_angle = 0;
     //
     while ( 1 )
     {
         // double delta_l = std::get<0>(chassis.get_average_encoders(l_id, r_id)) - prev_l_encoder;
         // double delta_r = std::get<1>(chassis.get_average_encoders(l_id, r_id)) - prev_r_encoder;
         // double delta_theta = chassis.calc_delta_theta(prev_angle, delta_l, delta_r);
         // prev_angle = prev_angle + delta_theta;
         // relative_angle = relative_angle + delta_theta;
         //

         std::string msg = (
             "[INFO] " + std::string("CHASSIS_PID ")
             + ", Actual_Vol1: " + std::to_string(Motors::front_left.get_actual_voltage())
             + ", Actual_Vol2: " + std::to_string(Motors::front_right.get_actual_voltage())
             + ", Actual_Vol3: " + std::to_string(Motors::back_left.get_actual_voltage())
             + ", Actual_Vol4: " + std::to_string(Motors::back_right.get_actual_voltage())
             + ", Slew: " + std::to_string(0)
             + ", Brake: " + std::to_string(Motors::front_left.get_brake_mode())
             + ", Gear: " + std::to_string(Motors::front_left.get_gearset())
             + ", I_max: " + std::to_string(I_max)
             + ", I: " + std::to_string(0)
             + ", kD: " + std::to_string(kD)
             + ", kI: " + std::to_string(kI)
             + ", kP: " + std::to_string(kP)
             + ", Time: " + std::to_string(pros::millis())
             + ", Position_Sp: " + std::to_string(1269.32)
             + ", position_l: " + std::to_string(Sensors::left_encoder.get_position(l_id))
             + ", position_r: " + std::to_string(Sensors::right_encoder.get_position(r_id))
             + ", Heading_Sp: " + std::to_string(0)
             + ", Relative_Heading: " + std::to_string(0)
             + ", Actual_Vel1: " + std::to_string(Motors::front_left.get_actual_velocity())
             + ", Actual_Vel2: " + std::to_string(Motors::front_right.get_actual_velocity())
             + ", Actual_Vel3: " + std::to_string(Motors::back_left.get_actual_velocity())
             + ", Actual_Vel4: " + std::to_string(Motors::back_right.get_actual_velocity())
         );
         // std::cout << msg << "\n";
         pros::delay(10);
     }
 }



 void Exit( int signal )
 {
     //Writer writer;
     std::cerr << "program caught " << signal << "\n" << std::flush;
     std::cerr << "errno: " << errno << "\n" << std::flush;
     std::cerr << "strerror: " << strerror(errno) << "\n" << std::flush;
     pros::delay(100); // wait for stdout to be flushed
     raise(signal);
 }



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    Server server;
    server.clear_stdin();
    server.start_server();
    server.set_debug_mode(true);


    std::cout << "opcontrol started\n";

    std::signal(SIGSEGV, Exit);
    std::signal(SIGTERM, Exit);
    std::signal(SIGINT, Exit);
    std::signal(SIGILL, Exit);
    std::signal(SIGABRT, Exit);
    std::signal(SIGFPE, Exit);
    std::signal(SIGBUS, Exit);
    std::signal(SIGALRM, Exit);
    std::signal(SIGSTOP, Exit);
    std::signal(SIGUSR1, Exit);
    std::signal(SIGUSR2, Exit);
    std::signal(SIGKILL, Exit);

    pros::delay(100);

    lv_scr_load(tempScreen::temp_screen);


    Controller controllers;
    DriverControlLCD lcd;

    Chassis chassis( Motors::front_left, Motors::front_right, Motors::back_left, Motors::back_right, Motors::mid_left, Motors::mid_right, Sensors::left_encoder, Sensors::right_encoder, 16, 3/5);
    PositionTracker* tracker = PositionTracker::get_instance();
    tracker->enable_imu();
    tracker->start_thread();


    Autons autons;
    // autons.run_autonomous();
    // autons.skills2();

    pros::Task driver_control_task (driver_control,
                                    (void*)NULL,
                                    TASK_PRIORITY_DEFAULT,
                                    TASK_STACK_DEPTH_DEFAULT,
                                    "DriverControlTask");

    int i = 0;
    uint8_t sins[360] = {
      127,129,131,134,136,138,140,143,145,147,149,151,154,156,158,160,162,164,166,169,171,173,175,177,179,181,183,185,187,189,191,193,195,196,198,200,
      202,204,205,207,209,211,212,214,216,217,219,220,222,223,225,226,227,229,230,231,233,234,235,236,237,239,240,241,242,243,243,244,245,246,247,248,
      248,249,250,250,251,251,252,252,253,253,253,254,254,254,254,254,254,254,255,254,254,254,254,254,254,254,253,253,253,252,252,251,251,250,250,249,
      248,248,247,246,245,244,243,243,242,241,240,239,237,236,235,234,233,231,230,229,227,226,225,223,222,220,219,217,216,214,212,211,209,207,205,204,
      202,200,198,196,195,193,191,189,187,185,183,181,179,177,175,173,171,169,166,164,162,160,158,156,154,151,149,147,145,143,140,138,136,134,131,129,
      127,125,123,120,118,116,114,111,109,107,105,103,100,98,96,94,92,90,88,85,83,81,79,77,75,73,71,69,67,65,63,61,59,58,56,54,
      52,50,49,47,45,43,42,40,38,37,35,34,32,31,29,28,27,25,24,23,21,20,19,18,17,15,14,13,12,11,11,10,9,8,7,6,
      6,5,4,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,4,5,
      6,6,7,8,9,10,11,11,12,13,14,15,17,18,19,20,21,23,24,25,27,28,29,31,32,34,35,37,38,40,42,43,45,47,49,50,
      52,54,56,58,59,61,63,65,67,69,71,73,75,77,79,81,83,85,88,90,92,94,96,98,100,103,105,107,109,111,114,116,118,120,123,125
    };

    pros::ADIDigitalIn limit_switch(pros::ext_adi_port_pair_t(EXPANDER_PORT, 'D'));
    int mode = 1;

    int r = 0;
    int g = 0;
    int b = 0;
    while(1) {
        if(limit_switch.get_new_press()) {
            mode += 1;
            if(mode > 2) {
                mode = 0;
            }
            r = 0;  // clear color values
            g = 0;
            b = 0;
        }

        switch(mode) {
            case 0: {  // rainbow cycle
                if(r > 255){
                   r=0;
                   g++;
                }
                if(g > 255){
                   g=0;
                   b++;
                }
                if(b > 255){
                   b=0;
                }
                r++;
                // Sensors::rgb_leds.set_color(r, g, b);
                i += 1;
                Sensors::rgb_leds.set_color(sins[i], sins[(i+120)%360], sins[(i+240)%360]);
                break;

            } case 1: {  // solid green
                Sensors::rgb_leds.set_color(0, 255, 0);
                break;

            } case 2: {  // off
                Sensors::rgb_leds.set_color(0, 0, 0);
                break;
            }
        }

        // Sensors::rgb_leds.set_color(sins[i], sins[(i+120)%360], sins[(i+240)%360]);
        // print encoder values

        // std::cout << "r: " << Sensors::right_encoder.get_position(r_id) << " | l: " << Sensors::left_encoder.get_position(l_id) << " | s: " << Sensors::strafe_encoder.get_position(s_id) << "\n";
        // double delta_theta = chassis.calc_delta_theta(prev_angle, ref_angle, Sensors::left_encoder.get_position(l_id) - prev_l, Sensors::right_encoder.get_position(r_id) - prev_r);
        // prev_angle = prev_angle + delta_theta;
        // prev_l = Sensors::left_encoder.get_position(l_id);
        // prev_r = Sensors::right_encoder.get_position(r_id);
        //
        // std::cout << "delta theta: " << delta_theta << "  |  new angle: " << prev_angle << "\n";
        // std::cout << tracker->get_position().x_pos << " " << tracker->get_position().y_pos << " " << tracker->to_degrees(tracker->get_position().theta) << "\n";
        lcd.update_labels();
        // server.handle_requests(50);
        // std::cout << "handling requests\n";
        // logger.dump();

        pros::delay(20);
    }
}
