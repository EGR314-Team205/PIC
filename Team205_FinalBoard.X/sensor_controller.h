/* 
 * File:   sensor_controller.h
 * Author: Michael Gross
 *
 * Created on April 14, 2023, 3:43 PM
 */

#ifndef SENSOR_CONTROLLER_H
#define	SENSOR_CONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#include "mcc_generated_files/mcc.h"
    
#define THRESH_CUTOFF 5
#define TRANSMIT_LENGTH 7

uint8_t tempData;
uint16_t hallRaw;
double windSpeed;

int threshCount;
float sensorThresh[2]; //{temp, wind speed} thresh value

void sensor_read(bool);

void set_thresh(int, float);

bool thresh_counter(void);

void data_transmit(char);

#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_CONTROLLER_H */

