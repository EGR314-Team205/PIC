#include "interrupt_handler.h"
#include "motor_control.h"


void Interrupt_Handler_Initialize(void){
    timer_ms = 0;
    timer_s = 0;
    readCount = 0;
    manualMode = false;
    
    INTERRUPT_GlobalInterruptEnable();     // Enable the Global Interrupts
    INTERRUPT_PeripheralInterruptEnable();     // Enable the Peripheral Interrupts

    TMR2_SetInterruptHandler(internal_clock);
    TMR4_SetInterruptHandler(sensor_read);
    IOCAF0_SetInterruptHandler(button_override); //button triggers manual override
    EUSART1_SetRxInterruptHandler(Rx1_ISR);
    
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

void sensor_read(void){
        tempData = tempRead();
        hallRaw = hallRead();
        windSpeed = windSpeedCalc(t_update(),3); // ms = s * 10^-3
}

void Rx1_ISR(void){
    EUSART1_Receive_ISR();
    if(EUSART1_is_rx_ready()){
        rxData = EUSART1_Read();
        
//        if(EUSART1_is_tx_ready()){
//            EUSART1_Write(rxData);
//        }
        GPIO_OUT5_SetOpenDrain();
        GPIO_OUT6_Toggle();
    }          
}

uint8_t Read_EUSART1_Buffer(void){
    return rxData;
}

void button_override(void){
    manualMode = !manualMode;
    actionTrigger();
}
