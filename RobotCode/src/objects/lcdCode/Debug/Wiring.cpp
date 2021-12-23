/**
 * @file: ./RobotCode/src/lcdCode/Debug/Wiring.cpp
 * @author: Aiden Carney
 * @reviewed_on: 10/15/2019
 * @reviewed_by: Aiden Carney
 *
 * @see: Wiring.hpp
 *
 * contains class that shows wiring configuration
 */

#include "../../../../include/main.h"
#include "../../../../include/api.h"

#include "../Styles.hpp"
#include "Wiring.hpp"

#include "../../motors/Motors.hpp"
#include "../../sensors/Sensors.hpp"


bool Wiring::cont = true;

Wiring::Wiring()
{
    cont = true;

//screen
    wiring_screen = lv_obj_create(NULL, NULL);
    lv_obj_set_style(wiring_screen, &gray);

//init back button
    //button
    btn_back = lv_btn_create(wiring_screen, NULL);
    lv_btn_set_style(btn_back, LV_BTN_STYLE_REL, &toggle_btn_released);
    lv_btn_set_style(btn_back, LV_BTN_STYLE_PR, &toggle_btn_pressed);
    lv_btn_set_action(btn_back, LV_BTN_ACTION_CLICK, btn_back_action);
    lv_obj_set_width(btn_back, 75);
    lv_obj_set_height(btn_back, 25);

    //label
    btn_back_label = lv_label_create(btn_back, NULL);
    lv_obj_set_style(btn_back_label, &heading_text);
    lv_label_set_text(btn_back_label, "Back");

//init title label
    title_label = lv_label_create(wiring_screen, NULL);
    lv_label_set_style(title_label, &heading_text);
    lv_obj_set_width(title_label, 440);
    lv_obj_set_height(title_label, 20);
    lv_label_set_align(title_label, LV_LABEL_ALIGN_CENTER);
    lv_label_set_text(title_label, "Wiring");

//init motor info label
    motor_info = lv_label_create(wiring_screen, NULL);
    lv_label_set_style(motor_info, &subheading_text);
    lv_obj_set_width(motor_info, 220);
    lv_obj_set_height(motor_info, 200);
    lv_label_set_align(motor_info, LV_LABEL_ALIGN_LEFT);

    std::string motors_text = (
            "back right      (? RPM) - " + std::to_string(BR_MOTOR) + "\n"
            "mid right       (? RPM) - " + std::to_string(MR_MOTOR) + "\n"
            "front right     (? RPM) - " + std::to_string(FR_MOTOR) + "\n"
            "back left       (? RPM) - " + std::to_string(BL_MOTOR) + "\n"
            "mid left        (? RPM) - " + std::to_string(ML_MOTOR) + "\n"
            "front left      (? RPM) - " + std::to_string(FL_MOTOR) + "\n"
            "lift1           (? RPM) - " + std::to_string(LIFT1_MOTOR) + "\n"
            "lift2           (? RPM) - " + std::to_string(LIFT2_MOTOR) + "\n"
    );

    lv_label_set_text(motor_info, motors_text.c_str());

//init motor info label
    sensors_info = lv_label_create(wiring_screen, NULL);
    lv_label_set_style(sensors_info, &subheading_text);
    lv_obj_set_width(sensors_info, 220);
    lv_obj_set_height(sensors_info, 200);
    lv_label_set_align(sensors_info, LV_LABEL_ALIGN_LEFT);

    std::string sensors_text = (
        std::string("right enc top     - ") + RIGHT_ENC_TOP_PORT + "\n" +
        "right enc bottom  - " + RIGHT_ENC_BOTTOM_PORT + "\n" +
        "left enc top      - " + LEFT_ENC_TOP_PORT + "\n" +
        "left enc bottom   - " + LEFT_ENC_BOTTOM_PORT + "\n" +
        "lift potentiometer- " + LIFT_POTENTIOMETER_PORT + "\n" +
        "mogo potentiometer- " + MOGO_POTENTIOMETER_PORT + "\n" +
        "middle detector   - " + DETECTOR_MIDDLE_PORT + "\n" +
        "bottom detector   - " + DETECTOR_BOTTOM_PORT + "\n" +
        "optical sensor    - " + std::to_string(OPTICAL_PORT) + "\n"
    );

    lv_label_set_text(sensors_info, sensors_text.c_str());

//set positions
    lv_obj_set_pos(btn_back, 30, 210);

    lv_obj_set_pos(title_label, 220, 5);

    lv_obj_set_pos(motor_info, 20, 25);
    lv_obj_set_pos(sensors_info, 300, 25);


}

Wiring::~Wiring()
{
    lv_obj_del(wiring_screen);
}


/**
 * sets cont to false signifying user wants to go back, main loop will exit
 */
lv_res_t Wiring::btn_back_action(lv_obj_t *btn)
{
    cont = false;
    return LV_RES_OK;
}


/**
 * waits for cont to be false which occurs when the user hits the back button
 */
void Wiring::debug()
{
    cont = true;

    lv_scr_load(wiring_screen);

    while ( cont )
    {
        pros::delay(100);
    }
}
