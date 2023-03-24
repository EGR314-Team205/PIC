/* 
 * File:   temp_sensor.h
 * Author: Michael Gross
 *
 * Created on March 23, 2023, 6:59 PM
 */

#ifndef TEMP_SENSOR_H
#define	TEMP_SENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif

#define TEMP_ADDRESS 0x48
#define TEMP_CONFIG 0x01
#define TEMP_REG 0x00
    
uint8_t tempRead(void);


#ifdef	__cplusplus
}
#endif

#endif	/* TEMP_SENSOR_H */

