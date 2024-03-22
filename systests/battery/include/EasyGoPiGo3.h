/* FILE: EasyGoPiGo3.h

   Instance Vars:
     speed: default 150 DPS
     left_eye_color
     right_eye_color
     left_encoder_target
     right_encoder_target

   Implemented Methods:

   TODO Methods:
     set_speed(speed_in_DPS=150)
     get_speed()
     forward(): drive forward - use set_speed() or default: 150 DPS
     backward(): drive backward
     stop(): 
     drive_cm(dist_cm, blocking=true)
     drive_inches(dist_inches, blocking=true)
     right() : pivot cw around right wheel
     left():  pivot ccw around left wheel
     spin_right(): spin in place clockwise
     spin_left(): spin in place counter-clockwise
     target_reached(left_tgt_degrees, right_tgt_degrees):  use to detect when to stop forward(), backward(), right(), left(), spin_right(), spin_left()
                                                           and for non-blocking drive_cm() or drive_inches()
     reset_encoders(): resets both encoders to 0
     read_encoders(out:left, out:right, in:units=CM/INCH/DEGREE)
     read_encoders_average(out:ave, in:units=CM/INCH/DEGREE)
     turn_degrees(in:deg, blocking=true):  left: negative degrees
     blinker_on(id:{LEFT,RIGHT}
     set_left_eye_color(R,G,B)
     set_right_eye_color(R,G,B)
     set_eye_color(R,G,B)
     open_left_eye()
     open_right_eye()
     open_eyes()
     close_left_eye()
     close_right_eye()
     close_eyes()
*/


#ifndef EasyGoPiGo3_h_
#define EasyGoPiGo3_h_


// #include <stdint.h>
// #include <stdlib.h>
#include <stdio.h>            // for printf
// #include <string.h>           // for strstr
// #include <sys/time.h>         // for clock_gettime
// #include <unistd.h>
// #include <stdexcept>
// #include <cmath>             // for pi
// #include <string>
// #include <iostream>
// #include <iomanip>
// #include <fstream>
// #include <sstream>
// #include <algorithm>

#include <GoPiGo3.h>

//  CONSTANTS
#define LEFT                        0
#define RIGHT                       1

#define CM                          0
#define INCH                        1
#define DEGREE                      2

class EasyGoPiGo3 : public GoPiGo3 {

public:
    EasyGoPiGo3();                  // default constructor

    // Instance Variables
    int DEFAULT_SPEED = 150;
    int NO_LIMIT_SPEED = 1000;
    int speed = DEFAULT_SPEED;

    void set_speed(int speed_in);

};  //end class EasyGoPiGo3


#endif //ifndef EasyGoPiGo3.h
