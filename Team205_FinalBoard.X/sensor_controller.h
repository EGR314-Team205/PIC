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

uint8_t tempData;
uint16_t hallRaw;
double windSpeed;

float tempCutoff;
float windCutoff;

void sensor_read(void);

void set_thresh(float*);

void data_transmit(char);

#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_CONTROLLER_H */

