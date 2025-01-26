/* FILE: EasyGoPiGo3.cpp
 *
 * Simplified C++ API for GoPiGo3
 *
 * Patterned after the Python EasyGoPiGo3.py
 */

#include <EasyGoPiGo3.h>
#include <GoPiGo3.h>
#include <stdio.h>

using namespace std;
using namespace chrono;

EasyGoPiGo3::EasyGoPiGo3(){
    set_speed(DEFAULT_SPEED);
    reset_encoders();
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


void EasyGoPiGo3::drive_cm(float dist, bool blocking){

    float dist_mm = dist * 10.0;

    // number of degrees each wheel needs to turn
    float WheelTurnDegrees = ((dist_mm / WHEEL_CIRCUMFERENCE) * 360.0);
    printf("\nWheelTurnDegrees: %.1f \n",WheelTurnDegrees);
    // get starting position of each motor
    uint32_t StartPositionLeft = GoPiGo3::get_umotor_encoder(MOTOR_LEFT);
    uint32_t StartPositionRight = GoPiGo3::get_umotor_encoder(MOTOR_RIGHT);
    printf("StartPositionLeft: %u \n",StartPositionLeft);
    printf("StartPositionRight: %u \n",StartPositionRight);
    

    printf("StartPositionLeft + WheelTurnDegrees: %d \n",(uint32_t)(StartPositionLeft+WheelTurnDegrees));
    printf("StartPositionRight + WheelTurnDegrees: %d \n",(int)(StartPositionRight+WheelTurnDegrees));
    // tell motor final encoder desired
    int32_t pos_left =(int32_t)(StartPositionLeft + WheelTurnDegrees);
    int32_t pos_right =(int32_t)(StartPositionRight + WheelTurnDegrees);
    printf("\nset_motor_position(pos_left: %d   pos_right: %d )\n", pos_left, pos_right);
    set_motor_position(MOTOR_LEFT, pos_left );
    set_motor_position(MOTOR_RIGHT, pos_right);

    if (blocking == true) {  // wait for completion if requested
        while ( target_reached(
                    StartPositionLeft + WheelTurnDegrees,
                    StartPositionRight + WheelTurnDegrees) == false
                ) {
                this_thread::sleep_for(milliseconds(100));
            }
        
    };

};


void EasyGoPiGo3::drive_inches(float dist, bool blocking){
    drive_cm(dist * 2.54, blocking);
};

void  EasyGoPiGo3::backward(){
    set_motor_dps(MOTOR_LEFT + MOTOR_RIGHT, NO_LIMIT_SPEED * -1);
};


/*  // my initial target_reached() 
    bool EasyGoPiGo3::target_reached(float left_target_degrees, float right_target_degrees){
    int tolerance = 5;  // +/- 5 whole degrees?? that's what DI chose
    int min_left_target = left_target_degrees - tolerance;
    int max_left_target = left_target_degrees + tolerance;
    int min_right_target = right_target_degrees - tolerance;
    int max_right_target = right_target_degrees + tolerance;
    printf("\ntarget_reached: min_left/right_target - left: %d  right: %d \n",min_left_target,min_right_target);
    printf("target_reached: max_left/right_target - left: %d  right: %d \n",max_left_target,max_right_target);

    int current_left_position = (int) get_umotor_encoder(MOTOR_LEFT);
    int current_right_position = (int) get_umotor_encoder(MOTOR_RIGHT);

    if (current_left_position > min_left_target and 
        current_left_position < max_left_target and
        current_right_position > min_right_target and
        current_right_position < max_right_target
       ) { 
        return true;    
       }
    else {
        printf("target_reached: current encoders - left: %d  right: %d \n",current_left_position,current_right_position);
        return false;
    }   

}; 

*/

// ChatGPT target_reached()
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

void EasyGoPiGo3::reset_encoders(bool blocking){

    // easygopigo3.py reset_encoders blocking does not actually wait till motors stop moving
    if (blocking){
        set_motor_power(MOTOR_LEFT + MOTOR_RIGHT,0);
        this_thread::sleep_for(milliseconds(25));
    }
    uint32_t left_target = get_umotor_raw_encoder(MOTOR_LEFT);
    offset_umotor_raw_encoder(MOTOR_LEFT, left_target);
    uint32_t right_target = get_umotor_raw_encoder(MOTOR_RIGHT);
    offset_umotor_raw_encoder(MOTOR_RIGHT, (int32_t) right_target);
};

array<unsigned int,2> EasyGoPiGo3::read_encoders(){
    array<unsigned int,2> enc_array;
    enc_array[0] = (unsigned int) get_umotor_encoder(MOTOR_LEFT);
    enc_array[1] = (unsigned int) get_umotor_encoder(MOTOR_RIGHT);
    return enc_array;
};

array<unsigned int,2> EasyGoPiGo3::read_raw_encoders(){
    array<unsigned int,2> enc_array;
    enc_array[0] = (unsigned int) get_umotor_raw_encoder(MOTOR_LEFT);
    enc_array[1] = (unsigned int) get_umotor_raw_encoder(MOTOR_RIGHT);
    return enc_array;
};
