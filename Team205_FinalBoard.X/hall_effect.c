#include "hall_effect.h"

uint16_t hallRead(void){
    return (uint16_t)(I2C1_Read1ByteRegister(HALL_ADDRESS,HALL_ANGLE1) << 8 | I2C1_Read1ByteRegister(HALL_ADDRESS,HALL_ANGLE2));
}

void hallRecord(double *time){
    hall_pos[0] = hall_pos[1];
    hall_time[0] = hall_time[1];
    
    hall_pos[1] = hallRead();
    hall_time[1] = (uint16_t)(*time);
}

double windSpeedCalc(double time, float power){ 
   hallRecord(&time);
   double dw_dt = (hall_pos[1] - hall_pos[0]) / (hall_time[1] - hall_time[0]); // measure angular change per unit time
   double linearVel = (dw_dt * 2.0 * M_PI * WIND_CUP_RADIUS) / (TICKS_PER_REV*pow(10, power)) ; // convert encoder ticks to Linear velocity
   return linearVel;
}

void hallInit(void){
    hall_pos[HALL_DATALENGTH-1] = hallRead(); //populate half the array
    hall_time[HALL_DATALENGTH-1] = 0; //populate half the array
}
