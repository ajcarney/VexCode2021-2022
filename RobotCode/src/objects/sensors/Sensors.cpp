/**
 * @file: ./RobotCode/src/objects/sensors/Sensors.cpp
 * @author: Aiden Carney
 * @reviewed_on: 2/29/2020
 * @reviewed_by: Aiden Carney
 *
 * @see: Sensors.hpp
 *
 * contains definitions for sensors and implementation for sensor class
 */

#include "Sensors.hpp"
#include "../motors/Motors.hpp"
#include "../../Configuration.hpp"
#include "../serial/Logger.hpp"


namespace Sensors
{
    Encoder right_encoder{RIGHT_ENC_TOP_PORT, RIGHT_ENC_BOTTOM_PORT, false};
    Encoder left_encoder{LEFT_ENC_TOP_PORT, LEFT_ENC_BOTTOM_PORT, false};
    Encoder strafe_encoder{STRAFE_ENC_TOP_PORT, STRAFE_ENC_BOTTOM_PORT, true};

//    pros::ADIDigitalIn lift_potentiometer{pros::ext_adi_port_pair_t(EXPANDER_PORT, 'A')};
//    pros::ADIDigitalIn mogo_potentiometer{pros::ext_adi_port_pair_t(EXPANDER_PORT, 'B')};


    pros::Imu imu{IMU_PORT};
    bool imu_is_calibrated = false;
    
    AnalogInSensor lift_potentiometer(pros::ext_adi_port_pair_t(EXPANDER_PORT, LIFT_POTENTIOMETER_PORT));
    AnalogInSensor mogo_potentiometer(pros::ext_adi_port_pair_t(EXPANDER_PORT, MOGO_POTENTIOMETER_PORT));

    RGBLedString rgb_leds{pros::ext_adi_port_pair_t(EXPANDER_PORT, LED_R), pros::ext_adi_port_pair_t(EXPANDER_PORT, LED_G), pros::ext_adi_port_pair_t(EXPANDER_PORT, LED_B)};


    void calibrate_imu() {
        bool calibrated = false;
        while(!calibrated) {  // block until imu is connected and calibrated
            imu.reset();  // calibrate imu
            while(imu.is_calibrating()) {
                pros::delay(10);
                calibrated = true;
            }
        }
        imu_is_calibrated = true;
    }

    void log_data() {
        Logger logger;
        log_entry entry;
        entry.content = ("[INFO], " + std::to_string(pros::millis())
            + ", Sensor Data"
            +  ", Right_Enc: " + std::to_string(right_encoder.get_absolute_position(false))
            +  ", Left_Enc: " + std::to_string(left_encoder.get_absolute_position(false))
        );
        entry.stream = "clog";
        logger.add(entry);
    }

    /**
     * takes the average of each side of the drive encoders
     * hopefully to reduce error of encoders
     * returns tuple of encoder values
     */
    std::tuple<double, double> get_average_encoders(int l_id, int r_id) {
        // use a weighted average to merge all encoders on the robot for a hopefully more accurate reading
        double left_encoder_val = (0 * Motors::front_left.get_encoder_position() * Motors::chassis_gear_ratio) + (0 * Motors::back_left.get_encoder_position() * Motors::chassis_gear_ratio) + (1 * Sensors::left_encoder.get_position(l_id));
        double right_encoder_val = (0 * Motors::front_right.get_encoder_position() * Motors::chassis_gear_ratio) + (0 * Motors::back_right.get_encoder_position() * Motors::chassis_gear_ratio) + (1 * Sensors::right_encoder.get_position(r_id));

        return {left_encoder_val, right_encoder_val};
    }


}
