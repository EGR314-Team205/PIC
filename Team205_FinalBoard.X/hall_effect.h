/* 
 * File:   hall_effect.h
 * Author: Michael Gross
 *
 * Created on March 23, 2023, 6:35 PM
 */

#ifndef HALL_EFFECT_H
#define	HALL_EFFECT_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "mcc_generated_files/i2c1_master.h"
       
#define HALL_ADDRESS 0x36
#define HALL_ANGLE1 0x0E
#define HALL_ANGLE2 0x0F
#define HALL_DATALENGTH 2
#define TICKS_PER_REV 4096
#define WIND_CUP_RADIUS 0.06

    
uint16_t hall_pos[HALL_DATALENGTH];
uint16_t hall_time[HALL_DATALENGTH];

void hallInit(void);

uint16_t hallRead(void);

void hallRecord(double*);

double windSpeedCalc(double, float);

#ifdef	__cplusplus
}
#endif

#endif	/* HALL_EFFECT_H */



