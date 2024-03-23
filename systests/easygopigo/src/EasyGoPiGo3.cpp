/* FILE: EasyGoPiGo3.cpp
 *
 * Simplified C++ API for GoPiGo3
 *
 * Patterned after the Python EasyGoPiGo3.py
 */

#include <EasyGoPiGo3.h>
#include <stdio.h>

using namespace std;
using namespace chrono;

EasyGoPiGo3::EasyGoPiGo3(){
    set_speed(DEFAULT_SPEED);
}

float EasyGoPiGo3::volt(){
    return get_voltage_battery();
};

void EasyGoPiGo3::set_speed(int speed_in){
    // note compiler enforces speed_in - try/catch not implemented
    speed = speed_in;
    set_motor_limits(MOTOR_LEFT + MOTOR_RIGHT, 0, speed);
};

void EasyGoPiGo3::set_speed(){
    speed = DEFAULT_SPEED;
    set_motor_limits(MOTOR_LEFT + MOTOR_RIGHT, 0, speed);
};

int EasyGoPiGo3::get_speed(){
    return speed;
};

void EasyGoPiGo3::get_speed(int &out){
    out=speed;
};

void  EasyGoPiGo3::reset_speed(){
    speed = DEFAULT_SPEED;
};

void  EasyGoPiGo3::stop(){
    set_motor_dps(MOTOR_LEFT + MOTOR_RIGHT, 0);
    this_thread::sleep_for(milliseconds(100));
    set_motor_power(MOTOR_LEFT + MOTOR_RIGHT, MOTOR_FLOAT);
    this_thread::sleep_for(milliseconds(100));
};


void  EasyGoPiGo3::forward(){
    set_motor_dps(MOTOR_LEFT + MOTOR_RIGHT, NO_LIMIT_SPEED);  // actual speed controlled by motor_limits dps
};

/*

// void drive_cm(float dist, bool blocking=true);

// void drive_inches(float dist, bool blocking=true);

*/

void  EasyGoPiGo3::backward(){
    set_motor_dps(MOTOR_LEFT + MOTOR_RIGHT, NO_LIMIT_SPEED * -1);
};


