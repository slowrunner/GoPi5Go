/*
 *  FILE: egpgtest.cpp
 *
 *  Results: calls each EasyGoPiGo3 method
 *
 */

#include <EasyGoPiGo3.h>   // for GoPiGo3
#include <stdio.h>     // for printf
#include <csignal>     // ctrl-c handling
#include <thread>         // for sleep
#include <chrono>         // for durations to input to sleep
#include <array>
#include <inttypes.h>   // to printf int32_t
#include <limits.h>    // UINT_MAX

using namespace std;
using namespace chrono;


float DIODE_DROP = 0.76;  // 0.56 when charging

EasyGoPiGo3 EGPG; // Create a EasyGoPiGo3 instance


void handle_ctrl_c(int s){
    EGPG.reset_all();
    printf("\n");
    exit(0);
}

void print_encoder_values(){
    array<unsigned int,2> encoder_array;
    encoder_array=EGPG.read_encoders();
    printf("encoders - left: %u  right: %u \n",encoder_array[0],encoder_array[1]);
    encoder_array=EGPG.read_raw_encoders();
    printf("raw_encoders - left: %u  right: %u \n",encoder_array[0],encoder_array[1]);
}
void print_info() {
    printf("\nWHEEL_BASE_WIDTH: %.3f \n",EGPG.WHEEL_BASE_WIDTH);
    printf("WHEEL_DIAMETER: %.3f \n",EGPG.WHEEL_DIAMETER);
    printf("WHEEL_BASE_CIRCUMFERENCE: %.3f \n",EGPG.WHEEL_BASE_CIRCUMFERENCE);
    printf("WHEEL_CIRCUMFERENCE: %.3f \n",EGPG.WHEEL_CIRCUMFERENCE);
    printf("MOTOR_GEAR_RATIO: %d \n",EGPG.MOTOR_GEAR_RATIO);
    printf("ENCODER_TICKS_PER_ROTATION: %d \n",EGPG.ENCODER_TICKS_PER_ROTATION);
    printf("MOTOR_TICKS_PER_DEGREE: %.3f \n",EGPG.MOTOR_TICKS_PER_DEGREE);
    
    printf("mm per encoder degrees: %.3f \n",EGPG.WHEEL_CIRCUMFERENCE / 360);  // encoders report 1deg precision
    // raw encoder reports in MOTOR_TICKS_PER_DEGREE precision i.e. 5.333 for 16-tick motors, or 2.0 for 6-tick motors 
    printf("mm per raw encoder tick: %.3f \n",EGPG.WHEEL_CIRCUMFERENCE / (360 * EGPG.MOTOR_TICKS_PER_DEGREE));
    print_encoder_values();
}

void test_volt() {
    printf("\n*** test_volt() ***\n");
    float reading = EGPG.volt();
    float adjusted_vBatt = reading + DIODE_DROP;
    printf("  Battery Reading: %.2f Adjusted: %.2f volts \n", reading, adjusted_vBatt);
}

void test_set_speed() {
    printf("\n*** test_set_speed() ***\n");
    printf("  Current speed: %d  \n",EGPG.get_speed());
    printf("  Setting set_speed(300)");
    EGPG.set_speed(300);
    printf("  Current speed: %d  \n",EGPG.get_speed());
    printf("  Reetting with set_speed() default value");
    EGPG.set_speed();
    printf("  Current speed: %d  \n",EGPG.get_speed());
}

void test_get_speed() {
    printf("\n*** test_get_speed() ***\n");
    int ispeed;
    EGPG.get_speed(ispeed);
    printf("  get_speed(): %d  get_speed(ispeed): %d \n",EGPG.get_speed(),ispeed);
}

void test_stop() {
    printf("\n*** test_stop() ***\n");
    EGPG.stop();
    printf("  Wheels should be floating \n");
}

void test_offset_raw_encoders(){

    printf("\n*** Test offset = urawposition\n");
    print_encoder_values();
    uint32_t urawoffset_left = EGPG.get_umotor_raw_encoder(MOTOR_LEFT);
    uint32_t urawoffset_right = EGPG.get_umotor_raw_encoder(MOTOR_RIGHT);
    printf("\n calling offset_umotor_raw_encoder(left, %u)", urawoffset_left);
    printf("\n calling offset_umotor_raw_encoder(right, %u)\n", urawoffset_right);
    int res = EGPG.offset_umotor_raw_encoder(MOTOR_LEFT,urawoffset_left);
    res = EGPG.offset_umotor_raw_encoder(MOTOR_RIGHT,urawoffset_right);
    print_encoder_values();


    printf("\nTest offset = 0\n");
    print_encoder_values();
    uint32_t urawoffset = 0;
    printf("\n calling offset_umotor_raw_encoder(both, %u)\n", urawoffset);
    res = EGPG.offset_umotor_raw_encoder(MOTOR_LEFT,urawoffset);
    res = EGPG.offset_umotor_raw_encoder(MOTOR_RIGHT,urawoffset);
    print_encoder_values();

    printf("\nTest offset = 1\n");
    print_encoder_values();
    urawoffset = 1;
    printf("\n calling offset_umotor_raw_encoder(both, %u)\n", urawoffset);
    res = EGPG.offset_umotor_raw_encoder(MOTOR_LEFT,urawoffset);
    res = EGPG.offset_umotor_raw_encoder(MOTOR_RIGHT,urawoffset);
    print_encoder_values();

 

}

void test_reset_encoders(){
    printf("\n*** Test reset_encoders()\n");
    print_encoder_values();
    printf("\n  calling EGPG.reset_encoders()\n");
    EGPG.reset_encoders();
    print_encoder_values();

    if (EGPG.get_umotor_raw_encoder(MOTOR_LEFT) == 0) {
        printf("\n  calling EGPG.offset_umotor_raw_encoder(left, UINT_MAX/2) to not be at 0 \n");
        EGPG.offset_umotor_raw_encoder(MOTOR_LEFT, UINT_MAX / 2);
    }
    if (EGPG.get_umotor_raw_encoder(MOTOR_RIGHT) == 0) {
        printf("  calling EGPG.offset_umotor_raw_encoder(right, UINT_MAX/2) to not be at 0 \n");
        EGPG.offset_umotor_raw_encoder(MOTOR_RIGHT,UINT_MAX / 2);
    }
    print_encoder_values();
    printf("\n  calling EGPG.reset_encoders()\n");
    EGPG.reset_encoders();
    print_encoder_values();
}

void test_forward() {
    printf("\n*** test_forward() ***\n");
    print_encoder_values();
    EGPG.forward();
    this_thread::sleep_for(milliseconds(1000));
    EGPG.stop();
    print_encoder_values();
    printf("  Wheels should be floating \n");
}

void test_backward() {
    printf("\n*** test_backward() ***\n");
    print_encoder_values();
    EGPG.backward();
    this_thread::sleep_for(milliseconds(1000));
    EGPG.stop();
    print_encoder_values();
    printf("  Wheels should be floating \n");
}

void test_drive_cm() {

    printf("\n*** test_drive_cm(10.0, non-blocking) ***\n");
    print_encoder_values();
    printf("\nresetting encoders\n");
    EGPG.reset_encoders(true);  //blocking
    print_encoder_values();

    exit(0);


    EGPG.drive_cm(10.0,false);
    printf("\ndrive_cm(10.0) in progress - ~2 sec - will force stop after 4 seconds\n");
    // 10cm at 150 DPS (about 0.055 m/s) should take about 2 sec - wait four.
    this_thread::sleep_for(milliseconds(4000));
    EGPG.stop();
    print_encoder_values();
    printf("\n  10 cm? \n");

    EGPG.stop();
    this_thread::sleep_for(milliseconds(5000));

    printf("\n*** test_drive_cm(-10.0, blocking) ***\n");
    print_encoder_values();
    printf("\nresetting encoders");
    EGPG.reset_encoders(true);  //blocking
    print_encoder_values();

    EGPG.drive_cm(-10.0,true);
    printf("\ndrive_cm(10.0) complete\n");
    this_thread::sleep_for(milliseconds(2000));
    print_encoder_values();
    EGPG.stop();


}

int main(){

  signal (SIGINT, handle_ctrl_c);

  // Make sure that the GoPiGo3 is communicating and that the firmware is compatible with the drivers.
  // pass 'false' to detect() to make the error non-critical (return the error instead of exiting the program).
  if(EGPG.detect(false) == ERROR_NONE){
    printf("\n*** Starting egpgtest at %.2f volts\n",EGPG.volt() + DIODE_DROP);
  }else{
    printf("\nError communicating with GoPiGo3\n");
    exit(1);
  }

  print_info();
  test_volt();
  test_get_speed();
  test_set_speed();
  test_stop();
  test_offset_raw_encoders();
  test_reset_encoders();

  exit(0);

  test_forward();
  EGPG.stop();
  this_thread::sleep_for(milliseconds(5000));
    
  test_backward();
  EGPG.stop();
  this_thread::sleep_for(milliseconds(5000));

  test_drive_cm();


  // DONE MAIN
  return 0;
}
