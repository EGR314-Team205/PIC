#include "temp_sensor.h"

uint8_t tempRead(void){
    return I2C1_Read1ByteRegister(TEMP_ADDRESS, TEMP_REG);
}
