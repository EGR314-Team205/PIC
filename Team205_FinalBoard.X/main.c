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


#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "mcc_generated_files/i2c1_master.h"

#include "hall_effect.h"
#include "motor_control.h"
#include "temp_sensor.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

void system_init(){
    TMR2_StartTimer();
    GPIO_OUT5_SetOpenDrain();
    hallInit();
    motorStop();
}

uint16_t timer_us = 0;
uint16_t timer_ms = 0;

void timer_callback(void){
    timer_us++;
    if (timer_us>=1000){
        timer_us=timer_us-1000;
        timer_ms++;
//        IO_RE0_Toggle();
    } 
}

float t_update(void){
    return (float)timer_ms + (float)timer_us /1000;
}


void main(void)
{

    SYSTEM_Initialize();     // Initialize the device
    system_init();
    
    INTERRUPT_GlobalInterruptEnable();     // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();     // Enable the Peripheral Interrupts
    
    TMR2_SetInterruptHandler(timer_callback);
    IOCAF0_SetInterruptHandler(motorTrigger);
   
    uint8_t temp_data;
    uint16_t hall_raw;
    double wind_speed;
    int readCount = 0;
    
    int threshCount = 0;
    
    uint8_t motor1_fault;
    uint8_t motor2_fault;
    
    
    while (1){
        
        temp_data = tempRead();
        hall_raw = hallRead();
        wind_speed = windSpeedCalc();
    
        if (MOTOR_FAULT1_GetValue()==0)
            motor1_fault = motorFaultRead(1);
        if (MOTOR_FAULT2_GetValue()==0)
            motor2_fault = motorFaultRead(2);
        
        if(wind_speed>10 && threshCount<5)
            threshCount++;
        else if (wind_speed>10 && threshCount>=5){
            motorTrigger();
            threshCount = 0;
        }
        else
            threshCount = 0;
            
        
        if(EUSART1_is_tx_ready()){
            printf("Reading%i(t=%.6u): \n\r", ++readCount, timer_ms);
                    printf("\tTemp=%uC \n\r", temp_data);
                    printf("\tHall=%u \n\r", hall_raw);
                    printf("\tWind Speed=%5.5d \n\r", wind_speed);
                    printf("\tMotor 1 Fault Code=%u \n\r", motor1_fault);
                    printf("\tMotor 2 Fault Code=%u\n\r", motor2_fault);
            GPIO_OUT6_Toggle();
            __delay_ms(10);
        }
    }
    
    motor1_fault = 0;
    motor2_fault = 0;
}