#include "motor_control.h"
#include "temp_sensor.h"
#include "hall_effect.h"
#include "sensor_controller.h"
#include "interrupt_handler.h"

void sensor_read(void){
        tempData = tempRead();
        hallRaw = hallRead();
        windSpeed = windSpeedCalc(t_update(),3); // ms = s * 10^-3
        data_transmit(';');
}

void set_thresh(float *thresh){
    //[temp, wind, button]
    tempCutoff = thresh[0];
    windCutoff = thresh[1];
}

void data_transmit(char delim){
    float data[] = {tempData, tempCutoff, (float)windSpeed, windCutoff};
    for(int i=0; i<2; i++){
        printf("%f%c",data[i],delim);
    }
    printf("\n\r");
    
}





