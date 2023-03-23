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
#include <math.h>
#include <string.h>
#include <stdio.h>

#define HALL_ADDRESS 0x36
#define HALL_ANGLE1 0x0E
#define HALL_ANGLE2 0x0F
#define HALL_DATALENGTH 2
#define TICKS_PER_REV 4096
#define WIND_CUP_RADIUS 0.015

#define TEMP_ADDRESS 0x48
#define TEMP_CONFIG 0x01
#define TEMP_REG 0x00

//MOTOR 1 = SOLENOID
#define MOTOR1_WRITE 0xD0
#define MOTOR1_READ 0xD1
#define MOTOR2_WRITE 0xC0
#define MOTOR2_READ 0xC1

//MOTOR 2 = DC MOTOR
#define MOTOR_CONTROL 0x00
#define MOTOR_FAULT 0x01
#define MOTOR_5V 0x3F
#define MOTOR_COAST 0b00
#define MOTOR_RVR 0b01
#define MOTOR_FWD 0b10
#define MOTOR_BREAK 0b11

uint16_t hall_pos[HALL_DATALENGTH];
uint16_t hall_time[HALL_DATALENGTH];

   bool deployStatus = false;


void motorController(uint8_t address, uint8_t speed, uint8_t dir){
    uint8_t data = speed << 2 | dir;
    I2C1_Write1ByteRegister(address, MOTOR_CONTROL, data);
}

void motorDeploy(){
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_RVR); //motor pivots umbrella backwards
    __delay_ms(2000);
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_FWD); //solenoid activates
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);//solenoid deactivates
    
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor pivots umbrella forward
    __delay_ms(2000);  
}

void motorStow(){
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor pivots umbrella forward
    __delay_ms(2000);
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_FWD); //solenoid activates
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);//solenoid deactivates
    
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_RVR); //motor pivots umbrella backward
    __delay_ms(2000);  
}

uint8_t motorFaultRead(uint8_t address){
    return I2C1_Read1ByteRegister(address, MOTOR_FAULT);
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

uint8_t tempRead(){
    return I2C1_Read1ByteRegister(TEMP_ADDRESS, TEMP_REG);
}

void system_init(){
    TMR2_StartTimer();
    GPIO_OUT5_SetOpenDrain();
    hall_pos[1] = hallRead(); //populate half the array
    hall_time[1] = TMR2_ReadTimer(); //populate half the array
    
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_COAST);
    
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

void motor_trigger(void){
    if(deployStatus){
        motorDeploy();
        deployStatus = true;
    }
    else{
        motorStow();
        deployStatus = false;
    }
}

void main(void)
{

    SYSTEM_Initialize();     // Initialize the device
    system_init();
    
    INTERRUPT_GlobalInterruptEnable();     // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();     // Enable the Peripheral Interrupts
    
    TMR2_SetInterruptHandler(timer_callback);
    IOCAF0_SetInterruptHandler(motor_trigger);
   
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
            motor1_fault = motorFaultRead(MOTOR1_READ);
        if (MOTOR_FAULT2_GetValue()==0)
            motor2_fault = motorFaultRead(MOTOR2_READ);
        
        if(wind_speed>10 && threshCount<5)
            threshCount++;
        else if (wind_speed>10 && threshCount>=5){
            motor_trigger();
            threshCount = 0;
        }
        else
            threshCount = 0;
            
        
        
        
        if(EUSART1_is_tx_ready()){
            printf("Reading%i: \n\r",++readCount);
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