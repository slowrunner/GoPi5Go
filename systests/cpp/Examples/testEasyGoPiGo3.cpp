/*
 * FILE: testEasyGoPoGo3.cpp
 *
 *
 */

#include <EasyGoPiGo3.h>
#include <stdio.h>          // for printf

EasyGoPiGo3 EGPG;   // Create an EasyGoPiGo3 instance

int main(){

    printf("\ntestEasyGoPiGo3:main: executing");
    printf("\ntestEasyGoPiGo3:main: EGPG.speed:%d",EGPG.speed);
    printf("\ntestEasyGoPiGo3:main: EGPG.WHEEL_BASE_WIDTH:%f",EGPG.WHEEL_BASE_WIDTH);
    printf("\n");
}
