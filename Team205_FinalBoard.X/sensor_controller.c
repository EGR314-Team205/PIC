#include "motor_control.h"
#include "temp_sensor.h"
#include "hall_effect.h"
#include "sensor_controller.h"
#include "interrupt_handler.h"

void sensor_read(bool conv){
        tempData = tempRead(conv);
        hallRaw = hallRead();
        windSpeed = windSpeedCalc(t_update(),1); // ms = s * 10^-3
        thresh_counter();
        data_transmit(';');
}

void set_thresh(int temp, float windSpeed){
    //[temp, wind, button]
    sensorThresh[0] = temp;
    sensorThresh[1] = windSpeed;
}

bool thresh_counter(void){
    bool triggerAdd = tempData >= sensorThresh[0] && windSpeed < sensorThresh[1];
    bool triggerReduce = windSpeed >= sensorThresh[1];
    
    if(triggerAdd && threshCount<THRESH_CUTOFF)
        threshCount++;
    else if (triggerReduce && threshCount>0){
        threshCount--;
    }
    return (threshCount==THRESH_CUTOFF);
}

void data_transmit(char delim){
    float data[TRANSMIT_LENGTH] = {tempData, sensorThresh[0], (float)windSpeed, sensorThresh[1], threshCount, THRESH_CUTOFF, (float)t_update()};
    for(int i=0; i<TRANSMIT_LENGTH; i++){
        printf("%.3f",data[i]);
        if(i!=TRANSMIT_LENGTH-1)
            printf("%c",delim);
            
    }
    printf("\n\r");
    
}





