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

using namespace std;
using namespace chrono;


float DIODE_DROP = 0.76;  // 0.56 when charging

EasyGoPiGo3 EGPG; // Create a EasyGoPiGo3 instance


void handle_ctrl_c(int s){
    printf("\n");
    exit(0);
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

void test_forward() {
    printf("\n*** test_forward() ***\n");
    EGPG.forward();
    this_thread::sleep_for(milliseconds(1000));
    EGPG.stop();
    printf("  Wheels should be floating \n");
}

void test_backward() {
    printf("\n*** test_backward() ***\n");
    EGPG.backward();
    this_thread::sleep_for(milliseconds(1000));
    EGPG.stop();
    printf("  Wheels should be floating \n");
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


  test_volt();
  test_get_speed();
  test_set_speed();
  test_stop();
  test_forward();
  test_backward();

  // DONE MAIN
  return 0;
}
