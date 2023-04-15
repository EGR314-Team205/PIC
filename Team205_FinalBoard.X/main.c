/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18LF26K40
        Driver Version    :  2.00
*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"

#include "hall_effect.h"
#include "motor_control.h"
#include "temp_sensor.h"
#include "interrupt_handler.h"
#include "sensor_controller.h"

#define THRESH_CUTOFF 5


/*
 * SYSTEM OVERVIEW:
 * 
 * PERIPHERAL:         TRIGGER:             RESULT:
 * TIMER2              1 MS                 INTERNAL CLOCK TIMER
 * TIMER3              2 MS                 SENSOR READ
 * EUSART1             RX DATA RECIEVE      STORE IN RXDATA
 * GPIO(PUSH_BUTTON)   PIN READ HIGH        TRIGGER MOTOR ACTION GROUP
 * 
 */


void PROJECT_INIT(){
    hallInit(); 
//    motorStop();
//    TMR3_StartTimer();
}


void main(void)
{
    SYSTEM_Initialize();     // Initialize the device
    PROJECT_INIT();
    Interrupt_Handler_Initialize();
    sensor_read();
    int readCount = 0;
    int threshCount = 0;
    const float initThresh[] = {25, 0.5};
    set_thresh(initThresh);
    
    printf("Staring...\n\r");  
    
    while (1){
        
      
//        motorFWD(); //debug motor
        uint8_t motor_fault = (!MOTOR_FAULT_GetValue() ? motorFaultRead() : 0);
        
        if(manualMode && windSpeed>2 && threshCount<THRESH_CUTOFF)
            threshCount++;
        else if (manualMode && windSpeed>2 && threshCount>=THRESH_CUTOFF){
            actionTrigger();
            threshCount = 0; //reset counter after triggering
        }
        else
            threshCount = 0; //reset due to system failure to keep over threshold
        
        if(EUSART1_is_tx_ready()){
            printf("Reading %i (t = %.3f s): \n\r", ++readCount, t_update());
                    printf("\tTemp: %u C \n\r", tempData);
                    printf("\tHall Effect Position: %u \n\r", hallRaw);
                    printf("\tWind Speed: %5.5f m/s \n\r", windSpeed);
                    printf("\tMotor Fault Codes: (%u) \n\r", motor_fault);
                    printf("\tTHRESH Count: %i/%i (Deploy State = %d)\n\r", threshCount, THRESH_CUTOFF, deployStatus);
                    printf("\tEUSART Data: %u \n\r",Read_EUSART1_Buffer());
                    
            __delay_ms(500);
        }
        
        
        //Threshold Indicators
        LED_DEBUG1_LAT = (tempData>=25);
        LED_DEBUG2_LAT = (hallRaw>2048);      
        LED_DEBUG3_LAT = !PUSH_BUTTON_GetValue();
    }
}