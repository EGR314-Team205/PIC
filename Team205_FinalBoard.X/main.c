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

void PROJECT_INIT(){
    TMR2_StartTimer();
    hallInit();
    motorStop();
}

uint16_t timer_ms = 0;
uint16_t timer_s = 0;

void timer_callback(void){
    timer_ms++;
    if (timer_ms>=1000){
        timer_ms-=1000;
        timer_s++;
//        IO_RE0_Toggle();
    } 
}

double t_update(void){
    return (double)timer_s + (double)timer_ms /1000;
}

volatile uint8_t rxData;
void Rx1_ISR(void){
    EUSART1_Receive_ISR();
    if(EUSART1_is_rx_ready()){
        rxData = EUSART1_Read();
        
//        if(EUSART1_is_tx_ready()){
//            EUSART1_Write(rxData);
//        }
        GPIO_OUT5_SetOpenDrain();
        GPIO_OUT6_Toggle();
    }          
}


void main(void)
{
    SYSTEM_Initialize();     // Initialize the device
//    PROJECT_INIT();
    
    INTERRUPT_GlobalInterruptEnable();     // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();     // Enable the Peripheral Interrupts
//    
    TMR2_SetInterruptHandler(timer_callback);
//    IOCAF0_SetInterruptHandler(actionTrigger); //button triggers manual override
    EUSART1_SetRxInterruptHandler(Rx1_ISR);
    
    uint8_t temp_data;
    uint16_t hall_raw;
    double wind_speed;
    int readCount = 0;
    
    int threshCount = 0;
    const int THRESH_CUTOFF = 5;
    
    uint8_t motor1_fault = 0;
    uint8_t motor2_fault = 0;
    
    printf("Staring...\n\r");
    while (1){

        
        temp_data = tempRead();
        hall_raw = hallRead();
        wind_speed = windSpeedCalc(t_update(),3); // ms = s * 10^-3
//        motorFWD(); //debug motor
//    
        if (MOTOR_FAULT1_GetValue()==0)
            motor1_fault = motorFaultRead(1);
        if (MOTOR_FAULT2_GetValue()==0)
            motor2_fault = motorFaultRead(2);
        
        if(wind_speed>2 && threshCount<THRESH_CUTOFF)
            threshCount++;
        else if (wind_speed>2 && threshCount>=THRESH_CUTOFF){
            actionTrigger();
            threshCount = 0;
        }
        else
            threshCount = 0;
        
        if(EUSART1_is_tx_ready()){
            printf("Reading %i (t=%.3f): \n\r", ++readCount, t_update());
                    printf("\tTemp: %u C \n\r", temp_data);
                    printf("\tHall Effect Position: %u \n\r", hall_raw);
                    printf("\tWind Speed: %5.5f m/s \n\r", wind_speed);
                    printf("\tMotor Fault Codes: (%u,%u) \n\r", motor1_fault, motor2_fault);
                    printf("\tTHRESH Count: %i/%i (Deploy State = %d)\n\r", threshCount, THRESH_CUTOFF, deployStatus);
                    printf("\tEUSART Data: %u \n\r",rxData);
            __delay_ms(500);
        }
        if(hall_raw>2048)
            LED_DEBUG2_SetHigh();
        else
            LED_DEBUG2_SetLow();
//        
        if(temp_data>=25)
            LED_DEBUG1_SetHigh();
        else
            LED_DEBUG1_SetLow();
//        
        motor1_fault = 0;
        motor2_fault = 0;
    }
}