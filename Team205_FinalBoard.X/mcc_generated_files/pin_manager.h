/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18LF26K40
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set PUSH_BUTTON aliases
#define PUSH_BUTTON_TRIS                 TRISAbits.TRISA0
#define PUSH_BUTTON_LAT                  LATAbits.LATA0
#define PUSH_BUTTON_PORT                 PORTAbits.RA0
#define PUSH_BUTTON_WPU                  WPUAbits.WPUA0
#define PUSH_BUTTON_OD                   ODCONAbits.ODCA0
#define PUSH_BUTTON_ANS                  ANSELAbits.ANSELA0
#define PUSH_BUTTON_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define PUSH_BUTTON_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define PUSH_BUTTON_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define PUSH_BUTTON_GetValue()           PORTAbits.RA0
#define PUSH_BUTTON_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define PUSH_BUTTON_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define PUSH_BUTTON_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define PUSH_BUTTON_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define PUSH_BUTTON_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define PUSH_BUTTON_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define PUSH_BUTTON_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define PUSH_BUTTON_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set MOTOR_FAULT1 aliases
#define MOTOR_FAULT1_TRIS                 TRISAbits.TRISA1
#define MOTOR_FAULT1_LAT                  LATAbits.LATA1
#define MOTOR_FAULT1_PORT                 PORTAbits.RA1
#define MOTOR_FAULT1_WPU                  WPUAbits.WPUA1
#define MOTOR_FAULT1_OD                   ODCONAbits.ODCA1
#define MOTOR_FAULT1_ANS                  ANSELAbits.ANSELA1
#define MOTOR_FAULT1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define MOTOR_FAULT1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define MOTOR_FAULT1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define MOTOR_FAULT1_GetValue()           PORTAbits.RA1
#define MOTOR_FAULT1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define MOTOR_FAULT1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define MOTOR_FAULT1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define MOTOR_FAULT1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define MOTOR_FAULT1_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define MOTOR_FAULT1_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define MOTOR_FAULT1_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define MOTOR_FAULT1_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set MOTOR_FAULT2 aliases
#define MOTOR_FAULT2_TRIS                 TRISAbits.TRISA2
#define MOTOR_FAULT2_LAT                  LATAbits.LATA2
#define MOTOR_FAULT2_PORT                 PORTAbits.RA2
#define MOTOR_FAULT2_WPU                  WPUAbits.WPUA2
#define MOTOR_FAULT2_OD                   ODCONAbits.ODCA2
#define MOTOR_FAULT2_ANS                  ANSELAbits.ANSELA2
#define MOTOR_FAULT2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define MOTOR_FAULT2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define MOTOR_FAULT2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define MOTOR_FAULT2_GetValue()           PORTAbits.RA2
#define MOTOR_FAULT2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define MOTOR_FAULT2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define MOTOR_FAULT2_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define MOTOR_FAULT2_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define MOTOR_FAULT2_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define MOTOR_FAULT2_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define MOTOR_FAULT2_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define MOTOR_FAULT2_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set LED_DEBUG1 aliases
#define LED_DEBUG1_TRIS                 TRISAbits.TRISA3
#define LED_DEBUG1_LAT                  LATAbits.LATA3
#define LED_DEBUG1_PORT                 PORTAbits.RA3
#define LED_DEBUG1_WPU                  WPUAbits.WPUA3
#define LED_DEBUG1_OD                   ODCONAbits.ODCA3
#define LED_DEBUG1_ANS                  ANSELAbits.ANSELA3
#define LED_DEBUG1_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define LED_DEBUG1_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define LED_DEBUG1_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define LED_DEBUG1_GetValue()           PORTAbits.RA3
#define LED_DEBUG1_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define LED_DEBUG1_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define LED_DEBUG1_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define LED_DEBUG1_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define LED_DEBUG1_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define LED_DEBUG1_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define LED_DEBUG1_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define LED_DEBUG1_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set LED_DEBUG2 aliases
#define LED_DEBUG2_TRIS                 TRISAbits.TRISA4
#define LED_DEBUG2_LAT                  LATAbits.LATA4
#define LED_DEBUG2_PORT                 PORTAbits.RA4
#define LED_DEBUG2_WPU                  WPUAbits.WPUA4
#define LED_DEBUG2_OD                   ODCONAbits.ODCA4
#define LED_DEBUG2_ANS                  ANSELAbits.ANSELA4
#define LED_DEBUG2_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define LED_DEBUG2_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define LED_DEBUG2_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define LED_DEBUG2_GetValue()           PORTAbits.RA4
#define LED_DEBUG2_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define LED_DEBUG2_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define LED_DEBUG2_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define LED_DEBUG2_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define LED_DEBUG2_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define LED_DEBUG2_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define LED_DEBUG2_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define LED_DEBUG2_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set GPIO_OUT1 aliases
#define GPIO_OUT1_TRIS                 TRISBbits.TRISB0
#define GPIO_OUT1_LAT                  LATBbits.LATB0
#define GPIO_OUT1_PORT                 PORTBbits.RB0
#define GPIO_OUT1_WPU                  WPUBbits.WPUB0
#define GPIO_OUT1_OD                   ODCONBbits.ODCB0
#define GPIO_OUT1_ANS                  ANSELBbits.ANSELB0
#define GPIO_OUT1_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define GPIO_OUT1_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define GPIO_OUT1_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define GPIO_OUT1_GetValue()           PORTBbits.RB0
#define GPIO_OUT1_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define GPIO_OUT1_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define GPIO_OUT1_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define GPIO_OUT1_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define GPIO_OUT1_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define GPIO_OUT1_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define GPIO_OUT1_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define GPIO_OUT1_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set GPIO_OUT2 aliases
#define GPIO_OUT2_TRIS                 TRISBbits.TRISB1
#define GPIO_OUT2_LAT                  LATBbits.LATB1
#define GPIO_OUT2_PORT                 PORTBbits.RB1
#define GPIO_OUT2_WPU                  WPUBbits.WPUB1
#define GPIO_OUT2_OD                   ODCONBbits.ODCB1
#define GPIO_OUT2_ANS                  ANSELBbits.ANSELB1
#define GPIO_OUT2_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define GPIO_OUT2_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define GPIO_OUT2_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define GPIO_OUT2_GetValue()           PORTBbits.RB1
#define GPIO_OUT2_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define GPIO_OUT2_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define GPIO_OUT2_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define GPIO_OUT2_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define GPIO_OUT2_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define GPIO_OUT2_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define GPIO_OUT2_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define GPIO_OUT2_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set GPIO_OUT3 aliases
#define GPIO_OUT3_TRIS                 TRISBbits.TRISB2
#define GPIO_OUT3_LAT                  LATBbits.LATB2
#define GPIO_OUT3_PORT                 PORTBbits.RB2
#define GPIO_OUT3_WPU                  WPUBbits.WPUB2
#define GPIO_OUT3_OD                   ODCONBbits.ODCB2
#define GPIO_OUT3_ANS                  ANSELBbits.ANSELB2
#define GPIO_OUT3_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define GPIO_OUT3_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define GPIO_OUT3_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define GPIO_OUT3_GetValue()           PORTBbits.RB2
#define GPIO_OUT3_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define GPIO_OUT3_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define GPIO_OUT3_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define GPIO_OUT3_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define GPIO_OUT3_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define GPIO_OUT3_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define GPIO_OUT3_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define GPIO_OUT3_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set GPIO_OUT4 aliases
#define GPIO_OUT4_TRIS                 TRISBbits.TRISB3
#define GPIO_OUT4_LAT                  LATBbits.LATB3
#define GPIO_OUT4_PORT                 PORTBbits.RB3
#define GPIO_OUT4_WPU                  WPUBbits.WPUB3
#define GPIO_OUT4_OD                   ODCONBbits.ODCB3
#define GPIO_OUT4_ANS                  ANSELBbits.ANSELB3
#define GPIO_OUT4_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define GPIO_OUT4_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define GPIO_OUT4_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define GPIO_OUT4_GetValue()           PORTBbits.RB3
#define GPIO_OUT4_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define GPIO_OUT4_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define GPIO_OUT4_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define GPIO_OUT4_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define GPIO_OUT4_SetPushPull()        do { ODCONBbits.ODCB3 = 0; } while(0)
#define GPIO_OUT4_SetOpenDrain()       do { ODCONBbits.ODCB3 = 1; } while(0)
#define GPIO_OUT4_SetAnalogMode()      do { ANSELBbits.ANSELB3 = 1; } while(0)
#define GPIO_OUT4_SetDigitalMode()     do { ANSELBbits.ANSELB3 = 0; } while(0)

// get/set GPIO_OUT5 aliases
#define GPIO_OUT5_TRIS                 TRISBbits.TRISB4
#define GPIO_OUT5_LAT                  LATBbits.LATB4
#define GPIO_OUT5_PORT                 PORTBbits.RB4
#define GPIO_OUT5_WPU                  WPUBbits.WPUB4
#define GPIO_OUT5_OD                   ODCONBbits.ODCB4
#define GPIO_OUT5_ANS                  ANSELBbits.ANSELB4
#define GPIO_OUT5_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define GPIO_OUT5_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define GPIO_OUT5_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define GPIO_OUT5_GetValue()           PORTBbits.RB4
#define GPIO_OUT5_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define GPIO_OUT5_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define GPIO_OUT5_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define GPIO_OUT5_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define GPIO_OUT5_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define GPIO_OUT5_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define GPIO_OUT5_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define GPIO_OUT5_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set GPIO_OUT6 aliases
#define GPIO_OUT6_TRIS                 TRISBbits.TRISB5
#define GPIO_OUT6_LAT                  LATBbits.LATB5
#define GPIO_OUT6_PORT                 PORTBbits.RB5
#define GPIO_OUT6_WPU                  WPUBbits.WPUB5
#define GPIO_OUT6_OD                   ODCONBbits.ODCB5
#define GPIO_OUT6_ANS                  ANSELBbits.ANSELB5
#define GPIO_OUT6_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define GPIO_OUT6_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define GPIO_OUT6_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define GPIO_OUT6_GetValue()           PORTBbits.RB5
#define GPIO_OUT6_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define GPIO_OUT6_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define GPIO_OUT6_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define GPIO_OUT6_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define GPIO_OUT6_SetPushPull()        do { ODCONBbits.ODCB5 = 0; } while(0)
#define GPIO_OUT6_SetOpenDrain()       do { ODCONBbits.ODCB5 = 1; } while(0)
#define GPIO_OUT6_SetAnalogMode()      do { ANSELBbits.ANSELB5 = 1; } while(0)
#define GPIO_OUT6_SetDigitalMode()     do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set RC3 procedures
#define RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_GetValue()              PORTCbits.RC3
#define RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define RC3_SetPullup()             do { WPUCbits.WPUC3 = 1; } while(0)
#define RC3_ResetPullup()           do { WPUCbits.WPUC3 = 0; } while(0)
#define RC3_SetAnalogMode()         do { ANSELCbits.ANSELC3 = 1; } while(0)
#define RC3_SetDigitalMode()        do { ANSELCbits.ANSELC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()              PORTCbits.RC4
#define RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define RC4_SetPullup()             do { WPUCbits.WPUC4 = 1; } while(0)
#define RC4_ResetPullup()           do { WPUCbits.WPUC4 = 0; } while(0)
#define RC4_SetAnalogMode()         do { ANSELCbits.ANSELC4 = 1; } while(0)
#define RC4_SetDigitalMode()        do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RC6_SetPullup()             do { WPUCbits.WPUC6 = 1; } while(0)
#define RC6_ResetPullup()           do { WPUCbits.WPUC6 = 0; } while(0)
#define RC6_SetAnalogMode()         do { ANSELCbits.ANSELC6 = 1; } while(0)
#define RC6_SetDigitalMode()        do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSELC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSELC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCAF0 pin functionality
 * @Example
    IOCAF0_ISR();
 */
void IOCAF0_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCAF0 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCAF0 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(MyInterruptHandler);

*/
void IOCAF0_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCAF0 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCAF0_SetInterruptHandler() method.
    This handler is called every time the IOCAF0 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(IOCAF0_InterruptHandler);

*/
extern void (*IOCAF0_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCAF0 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCAF0_SetInterruptHandler() method.
    This handler is called every time the IOCAF0 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(IOCAF0_DefaultInterruptHandler);

*/
void IOCAF0_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/