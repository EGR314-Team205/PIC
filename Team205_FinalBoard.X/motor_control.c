#include "motor_control.h"


void motorController(uint8_t address, uint8_t speed, uint8_t dir){
    uint8_t data = speed << 2 | dir;
    I2C1_Write1ByteRegister(address, MOTOR_CONTROL, data);
}

void motorDeploy(){
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_RVR); //motor pivots umbrella backwards
    __delay_ms(2000);
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_FWD); //solenoid activates
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);//solenoid deactivates
    
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor pivots umbrella forward
    __delay_ms(2000);  
}

void motorStow(){
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_FWD); //motor pivots umbrella forward
    __delay_ms(2000);
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_FWD); //solenoid activates
    motorController(MOTOR1_WRITE, MOTOR_5V, MOTOR_COAST);//solenoid deactivates
    
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_RVR); //motor pivots umbrella backward
    __delay_ms(2000);  
}

void motorTrigger(void){
    if(deployStatus){
        motorDeploy();
        deployStatus = true;
    }
    else{
        motorStow();
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
    motorController(MOTOR2_WRITE, MOTOR_5V, MOTOR_COAST);
}
