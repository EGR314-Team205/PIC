#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "mcc_generated_files/i2c1_master.h"
/*
                Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    I2C1_Initialize();
    uint8_t read_var[2];
    uint16_t check;
    float A = 11.375;
    float degree = 0;
   
    
    while (1)
    {
        read_var[0] = I2C1_Read1ByteRegister(0x36, 0x0E);
        read_var[1] = I2C1_Read1ByteRegister(0x36, 0x0F);
        check = read_var[0] << 8 | read_var[1];
        printf("Angle = %u \n \r",(uint16_t) check);
        __delay_ms(50);
        
        
        read_var[0] = I2C1_Read1ByteRegister(0x36, 0x1B);
        read_var[1] = I2C1_Read1ByteRegister(0x36, 0x1C);
        check = read_var[0] << 8 | read_var[1];
        printf("Mag = %u \n \r \n", check);
        __delay_ms(50);
    }
}