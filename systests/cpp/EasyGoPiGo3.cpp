/* FILE: EasyGoPiGo3.cpp
 *
 * Simplified C++ API for GoPiGo3
 *
 * Patterned after the Python EasyGoPiGo3.py
 */

#include <EasyGoPiGo3.h>

EasyGoPiGo3::EasyGoPiGo3(){
    set_speed(DEFAULT_SPEED);
}

void EasyGoPiGo3::set_speed(int speed_in){
    speed = speed_in;
};
