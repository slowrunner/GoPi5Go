// easygopigo3.h
#ifndef EASYGOPIGO3_H
#define EASYGOPIGO3_H

#include "GoPiGo3.h"
#include <string>
#include <vector>
#include <iostream>

class EasyGoPiGo3 : public GoPiGo3 {
public:
    EasyGoPiGo3();

    // Motor Control Methods
    void set_motor_dps(uint8_t port, int16_t dps);
    void drive_cm(float dist, float blocking = true);
    void drive_inches(float dist, float blocking = true);
    void drive_degrees(float degrees, float blocking = true);
    void stop();
    bool target_reached(int32_t left_target_degrees, int32_t right_target_degrees);

    // LED Control Methods
    void set_eye_color(uint8_t red, uint8_t green, uint8_t blue);
    void open_eyes();
    void close_eyes();

    // System Utility Methods
    std::string get_manufacturer();
    std::string get_board();
    float get_voltage();
    float get_battery_voltage();

private:
    void delay_ms(uint16_t ms);
};

#endif
