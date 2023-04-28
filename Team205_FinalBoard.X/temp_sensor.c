#include "temp_sensor.h"


float tempRead(bool convert){
    uint8_t data = I2C1_Read1ByteRegister(TEMP_ADDRESS, TEMP_REG);
    return (convert ? (float)data*(1.8) + 32 : (float)data);
}