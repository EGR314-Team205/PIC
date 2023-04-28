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


/*
 * SYSTEM OVERVIEW:
 * 
 *                          INTERRUPTS
 * 
 * PERIPHERAL:              TRIGGER:                    RESULT:
 * TIMER2                   1 MS                        INTERNAL CLOCK TIMER
 * TIMER4 [DISABLED]        10 MS                       SENSOR READ
 * EUSART1                  N/A                         PRINT TO DEBUG PC (DIRECT CONNECTION)
 * EUSART2                  RX DATA RECIEVE(ESP32)      STORE IN RXDATA
 * GPIO(PUSH_BUTTON)        PIN READ HIGH               TRIGGER MOTOR ACTION GROUP
 * 
 *                     
 *                          HEADER FILES
 * 
 * NAME:                    PURPOSE:
 * INTERRUPT HANDLER        CONTAINS TIMER, EUSART, AND GPIO INTERRUPTS
 * MOTOR CONTROLLER         CONTROLS DRIECTION OF DC MOTOR AND SOLENOID
 * HALL EFFECT              READS HALL EFFECT DATA, CALCULATES WIND SPEED
 * TEMP SENSOR              READS TEMPERATURE DATA
 * SENSOR CONTROLLER        OPERATES HALL EFFECT AND TEMP SENSOR FUNCTIONS, RETURNS INFORMATION IN ESP32 STRING PARSING FORMAT
 *                      
 * 
 */


void main(void){
    
    SYSTEM_Initialize();     // Initialize the device
    
    INTERRUPT_GlobalInterruptEnable();     // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();     // Enable the Peripheral Interrupts

    TMR2_SetInterruptHandler(internal_clock);
//    TMR4_SetInterruptHandler(sensor_read);
    IOCAF0_SetInterruptHandler(button_override); //button triggers manual override
    EUSART1_SetRxInterruptHandler(Rx1_ISR);
    
    hallInit();
    
    TMR2_StartTimer();
    TMR4_StartTimer();
    
    Interrupt_Handler_Initialize();
    set_thresh(25, 0.5); // set temp and wind speed cutoffs [default = (25, 0.5)]
    
    GPIO_OUT3_SetOpenDrain();
    
    while (1){
//        motorTRFWD(); //debug motor
//        solTrigger(); //debug solenoid
    sensor_read(tempConvert);
        /*  Threshold LED Indicators    */
        LED_DEBUG1_LAT = (tempData >= sensorThresh[0]);
        LED_DEBUG2_LAT = (hallRaw >= TICKS_PER_REV/2);      
        LED_DEBUG3_LAT = !PUSH_BUTTON_GetValue();
        GPIO_OUT4_LAT = tempConvert;
        
        __delay_ms(100);

    }
}