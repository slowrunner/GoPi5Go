/*
 *  FILE: batteryLife.cpp
 *
 *  Purpose: Loops until Pi5 dies,
 *           Prints battery reading and 0.76v diode drop adjusted voltage to console,
 *           writes datetime and diode drop adjussted voltage to ../batteryLife.csv
 *
 *  Results: With Oak-D-Lite and LIDAR powered at idle, Raspberry Pi5, GoPiGo3
 *
 *           Ran 3h 48m to battery protected shutdown
 *
 *           Diode-Drop is 0.76 off the dock, 0.56 on the dock
 *
 *  2024-03-22 02:02:34 Battery Reading: 11.77 Adjusted: 12.33 volts - Docked (corrected)
 *  2024-03-22 02:02:40 Battery Reading: 11.01 Adjusted: 11.77 volts - Undocked
 *  2024-03-22 02:02:46 Battery Reading: 11.02 Adjusted: 11.78 volts
 *  ...
 *  2024-03-22 06:03:49 Battery Reading: 7.45 Adjusted: 8.21 volts - shutdown
 *
 *  REQ: EaxyGoPiGo3 C++ API
 */

#include <EasyGoPiGo3.h>
#include <GoPiGo3.h>   // for GoPiGo3
#include <stdio.h>     // for printf
#include <string>
#include <fstream>
#include <vector>
#include <utility>  // std::pair
#include <chrono>   // time span
#include <thread>   // sleep
#include <csignal>
#include <sstream>
#include <iomanip>  // put_time

using namespace std;
using namespace chrono;


float DIODE_DROP = 0.76;  // 0.56 when on dock charging

EasyGoPiGo3 EGPG; // Create an EasyGoPiGo3 instance



bool keepLooping = true;
void handle_ctrl_c(int s){
        printf("\n");
        keepLooping = false;
}


int main(){

  signal (SIGINT, handle_ctrl_c);

  if(EGPG.detect(false) == ERROR_NONE){

    printf("\n*** Starting batteryLife at %.2f volts\n",EGPG.volt() + DIODE_DROP);
  }else{
    printf("\nError communicating with GoPiGo3\n");
    exit(1);
  }

  float reading = EGPG.volt();
  float adjusted_vBatt = reading + DIODE_DROP;

  // Create an output filestream object
  std::ofstream csvFile("batteryLife.csv", std::ios::app );
  csvFile << "\"Date Time        \",\"Adjusted vBatt\"" << endl ;

  chrono::milliseconds timespan(6000);
  stringstream sstimenow;

  do {

      reading = EGPG.volt();
      adjusted_vBatt = reading + DIODE_DROP;
      auto now = system_clock::now();
      auto timenow = system_clock::to_time_t(now);
      csvFile << put_time(localtime(&timenow), "%Y-%m-%d %X") << "," << adjusted_vBatt << endl;
      sstimenow.str("");      //clear out any old value
      sstimenow << put_time(localtime(&timenow), "%Y-%m-%d %X");
      cout << sstimenow.str();
      printf(" Battery Reading: %.2f Adjusted: %.2f volts\n", reading, adjusted_vBatt);
      this_thread::sleep_for(timespan);
  } while (keepLooping);


  // Close the file
  csvFile.close();






  return 0;





}
