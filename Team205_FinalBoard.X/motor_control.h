/* 
 * File:   motor_control.h
 * Author: Michael Gross
 *
 * Created on March 23, 2023, 6:26 PM
 */

#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "mcc_generated_files/i2c1_master.h"
    
#define MOTOR_WRITE 0xC0
#define MOTOR_READ 0xC1
    
#define MOTOR_CONTROL 0x00
#define MOTOR_FAULT 0x01
#define MOTOR_5V 0x3F
#define MOTOR_COAST 0b00
#define MOTOR_RVR 0b01
#define MOTOR_FWD 0b10
#define MOTOR_BREAK 0b11
    
bool deployStatus;
    
void motorController(uint8_t, uint8_t, uint8_t);

void solTrigger(void);

void motorFWDStep(void);

void motorFWD(void);

void motorRVR(void);

void motorOFF(void);

void umbDeploy(void);

void umbStow(void);

uint8_t motorFaultRead(void);

void actionTrigger(void);

void motorStop(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_CONTROL_H */

