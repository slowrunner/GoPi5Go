// easygopigo3.cpp
#include "easygopigo3.h"
#include <chrono>
#include <thread>

EasyGoPiGo3::EasyGoPiGo3() : GoPiGo3() {}

void EasyGoPiGo3::set_motor_dps(uint8_t port, int16_t dps) {
    set_motor_limits(port, 0, dps);
}

void EasyGoPiGo3::drive_cm(float dist, float blocking) {
    float wheel_circumference = WHEEL_DIAMETER * M_PI;
    int target_degrees = static_cast<int>((dist / wheel_circumference) * 360.0);
    set_motor_position_relative(MOTOR_LEFT + MOTOR_RIGHT, target_degrees);
    if (blocking) {
        while (!target_reached(target_degrees, target_degrees)) {
            delay_ms(10);
        }
    }
}

void EasyGoPiGo3::drive_inches(float dist, float blocking) {
    drive_cm(dist * 2.54, blocking);
}

void EasyGoPiGo3::drive_degrees(float degrees, float blocking) {
    float dist = (degrees / 360.0) * (WHEEL_BASE_WIDTH * M_PI);
    drive_cm(dist, blocking);
}

void EasyGoPiGo3::stop() {
    set_motor_power(MOTOR_LEFT + MOTOR_RIGHT, 0);
}

bool EasyGoPiGo3::target_reached(int32_t left_target_degrees, int32_t right_target_degrees) {
    int32_t left_position;
    int32_t right_position;
    get_motor_position(MOTOR_LEFT, left_position);
    get_motor_position(MOTOR_RIGHT, right_position);

    const int32_t tolerance = 5; // Tolerance in degrees
    bool left_within_target = (left_position >= left_target_degrees - tolerance && left_position <= left_target_degrees + tolerance);
    bool right_within_target = (right_position >= right_target_degrees - tolerance && right_position <= right_target_degrees + tolerance);

    return left_within_target && right_within_target;
}

void EasyGoPiGo3::set_eye_color(uint8_t red, uint8_t green, uint8_t blue) {
    set_led(LED_EYE_LEFT + LED_EYE_RIGHT, red, green, blue);
}

void EasyGoPiGo3::open_eyes() {
    set_eye_color(255, 255, 255);
}

void EasyGoPiGo3::close_eyes() {
    set_eye_color(0, 0, 0);
}

std::string EasyGoPiGo3::get_manufacturer() {
    char str[20];
    get_manufacturer(str);
    return std::string(str);
}

std::string EasyGoPiGo3::get_board() {
    char str[20];
    get_board(str);
    return std::string(str);
}

float EasyGoPiGo3::get_voltage() {
    return get_voltage_5v();
}

float EasyGoPiGo3::get_battery_voltage() {
    return get_voltage_battery();
}

void EasyGoPiGo3::delay_ms(uint16_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/*
Python methods that require easysensors:

# The following methods were not ported to C++:
# def init_distance_sensor(self):
# def init_light_sensor(self):
# def init_servo(self):
# def init_motor(self):
# def init_buzzer(self):
# Please refer to the original Python implementation for these methods.
*/
