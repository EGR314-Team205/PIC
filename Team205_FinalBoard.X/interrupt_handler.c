#include "interrupt_handler.h"
#include "motor_control.h"
#include "sensor_controller.h"


void Interrupt_Handler_Initialize(void){
    timer_ms = 0;
    timer_s = 0;
    manualMode = false;
    threshCount = 0;
    
}

void internal_clock(void){
    timer_ms++;
    if (timer_ms>=1000){
        timer_ms-=1000;
        timer_s++;
//        IO_RE0_Toggle();
    }
}

double t_update(void){
    return (double)timer_s + (double)timer_ms /1000;
}

void Rx1_ISR(void){
    EUSART1_Receive_ISR();
    if(EUSART1_is_rx_ready()){
        rxData = EUSART1_Read();
//        int rxSize = sizeof(rxData);
        if(rxData == 0x46 && !tempConvert){ // "F"
            sensorThresh[0] = sensorThresh[0]*(9/5)+32;
            tempConvert = true;
        }
            
        if(rxData == 0x43 && tempConvert){ // "C"
            sensorThresh[0] =  (sensorThresh[0]-32)*(5/9);
            tempConvert = false;
        }

    }

//        if(EUSART1_is_tx_ready()){
//            EUSART1_Write(rxData);
//        }
        GPIO_OUT5_SetOpenDrain();
        GPIO_OUT6_Toggle();    
}

uint8_t Read_EUSART1_Buffer(void){
    return rxData;
}

void button_override(void){
    manualMode = !manualMode;
    actionTrigger();
}
