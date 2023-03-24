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

#include <math.h>
       
#define HALL_ADDRESS 0x36
#define HALL_ANGLE1 0x0E
#define HALL_ANGLE2 0x0F
#define HALL_DATALENGTH 2
#define TICKS_PER_REV 4096
#define WIND_CUP_RADIUS 0.015

    
uint16_t hall_pos[HALL_DATALENGTH];
uint16_t hall_time[HALL_DATALENGTH];

void hallInit(void);

uint16_t hallRead();

void hallRecord();

double windSpeedCalc();

#ifdef	__cplusplus
}
#endif

#endif	/* HALL_EFFECT_H */


