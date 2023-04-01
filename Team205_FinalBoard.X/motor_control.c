#include "motor_control.h"


void motorController(uint8_t address, uint8_t speed, uint8_t dir){
    uint8_t data = speed << 2 | dir;
    I2C1_Write1ByteRegister(address, MOTOR_CONTROL, data);
}

void solTrigger(void){
        motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_FWD); //solenoid activates
        motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);//solenoid deactivates
}
void motorFWDStep(void){
        motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor forwards
        motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_COAST);//motor off
}
void motorFWD(void){
        motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor forwards
}

void motorRVR(void){
        motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_RVR); //motor backwards
}

void motorOFF(void){
        motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_COAST);//motor off
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

uint8_t motorFaultRead(int motor_num){
    switch(motor_num){
        case 1:
            return I2C1_Read1ByteRegister(MOTOR1_READ, MOTOR_FAULT);
        case 2:
            return I2C1_Read1ByteRegister(MOTOR1_READ, MOTOR_FAULT);
        default:
            return 0x0;
    }
    
}

void motorStop(void){
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);
    motorOFF();
}
