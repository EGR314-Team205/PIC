/* 
 * File:   interrupt_handler.h
 * Author: Michael Gross
 *
 * Created on April 4, 2023, 6:23 PM
 */

#ifndef INTERRUPT_HANDLER_H
#define	INTERRUPT_HANDLER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"

uint16_t timer_ms;
uint16_t timer_s;

uint8_t tempData;
uint16_t hallRaw;
double windSpeed;
int readCount;
    

volatile uint8_t rxData;

bool manualMode;

    
void Interrupt_Handler_Initialize(void);

void internal_clock(void);
double t_update(void);

uint16_t sensor_read(void);

void Rx1_ISR(void);

void button_override(void);

uint8_t Read_EUSART1_Buffer(void);


#ifdef	__cplusplus
}
#endif

#endif	/* INTERRUPT_HANDLER_H */

