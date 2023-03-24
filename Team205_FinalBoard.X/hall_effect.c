#include "hall_effect.h"

void hallInit(void){
    hall_pos[1] = hallRead(); //populate half the array
    hall_time[1] = 0; //populate half the array
}

uint16_t hallRead(){
    return (I2C1_Read1ByteRegister(HALL_ADDRESS,0x0E) << 8 | I2C1_Read1ByteRegister(HALL_ADDRESS,0x0F));
}

void hallRecord(){
    hall_pos[0] = hall_pos[1];
    hall_time[0] = hall_time[1];
    
    hall_pos[1] = hallRead();
    hall_time[1] = TMR2_ReadTimer();
}

double windSpeedCalc(){ 
   hallRecord();
   double dw_dt = (hall_pos[1] - hall_pos[0]) / (hall_time[1] - hall_time[0]); // measure angular change per unit time
   double linearVel = (dw_dt * 2.0 * M_PI * WIND_CUP_RADIUS) / TICKS_PER_REV ; // convert encoder ticks to Linear velocity
   return linearVel;
}
