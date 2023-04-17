#include "motor_control.h"


void motorController(uint8_t address, uint8_t speed, uint8_t dir){
    uint8_t data = (uint8_t)(speed << 2 | dir);
    I2C1_Write1ByteRegister(address, MOTOR_CONTROL, data);
}

void solTrigger(void){
    SOL_CONTROL_LAT = 1;
    __delay_us(1000);
    SOL_CONTROL_LAT = 0;
}
void motorFWDStep(void){
        motorController(MOTOR_WRITE, MOTOR_5V, MOTOR_FWD); //motor forwards
        motorController(MOTOR_WRITE, MOTOR_5V, MOTOR_COAST);//motor off
}
void motorFWD(void){
        motorController(MOTOR_WRITE, MOTOR_5V, MOTOR_FWD); //motor forwards
}

void motorRVR(void){
        motorController(MOTOR_WRITE, MOTOR_5V, MOTOR_RVR); //motor backwards
}

void motorOFF(void){
        motorController(MOTOR_WRITE, MOTOR_5V, MOTOR_COAST);//motor off
}

void umbDeploy(){
    motorRVR(); //motor pivots umbrella backwards
    __delay_ms(2000);
    motorOFF();
    solTrigger();
    motorFWD(); //motor pivots umbrella forward
    __delay_ms(2000);  
}

void umbStow(){
    motorFWD(); //motor pivots umbrella forward
    __delay_ms(2000);
    motorOFF();
    solTrigger();
    motorRVR(); //motor pivots umbrella backward
    __delay_ms(2000);  
}

void actionTrigger(void){
    if(deployStatus){
        umbDeploy();
        deployStatus = true;
    }
    else{
        umbStow();
        deployStatus = false;
    }
}

uint8_t motorFaultRead(void){
    return I2C1_Read1ByteRegister(MOTOR_READ, MOTOR_FAULT);  
}