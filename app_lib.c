#include <stdbool.h>

/* Defines IIC parameters */
#include"iic_defines.h"

/* IIC RPDL function definitions */
#include "r_pdl_iic.h"

/* General RPDL function definitions */
#include "r_pdl_definitions.h"

#include "app_lib.h"

/* Defines LCD functions used in this file */
#include "lcd.h"

#include <string.h>
#include <stdio.h>


signed char i2c_write(unsigned char slave_addr,
					  unsigned char reg_addr,
                      unsigned char length,
                      unsigned char const *data)
{
	bool err = true;
	unsigned char slaveAddress 		= slave_addr;
	unsigned char writeData[20]		= {0};
	unsigned char byte				= 0;
	
	writeData[0] = reg_addr;
	
	for(byte = 0; byte < (length); byte ++)
	{
		writeData[byte + 1] = data[byte];
	}
	
	err &= R_IIC_MasterSend(0,
							PDL_IIC_START_ENABLE | PDL_IIC_STOP_ENABLE,
					 		(slaveAddress << 1 | 0x00),
					 		(uint8_t*)writeData,
					 		length + 1,
					 		PDL_NO_FUNC,
					 		0);
	while (!err);
	
	if (err == 1)
		return 0;
	else
		return -1;
}


signed char i2c_read(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char *data)
{
	bool err = true;
	unsigned char slaveAddress		= slave_addr;
	unsigned char registerAddress	= reg_addr;


    if(registerAddress != 0xFF)
    {
		err &= R_IIC_MasterSend(IIC_CHANNEL, 
							PDL_IIC_START_ENABLE | PDL_IIC_STOP_DISABLE,
							(slaveAddress << 1 | 0x00), 
							&registerAddress,
							1, 
							PDL_NO_FUNC, 
							0);
		while (!err);						 
	}
	 
    err &= R_IIC_MasterReceive( IIC_CHANNEL, 
								PDL_NO_DATA, 
								(slaveAddress << 1 | 0x01), 
								(uint8_t*)data, 
                                length, 
								PDL_NO_FUNC, 
								0);
	while (!err);
	if (err == 1)
		return 0;
	else
		return -1;

}


int delay_ms(unsigned char delay)
{
	bool err = true;
	float ms = delay * (1E-3);
	
	
	/*uint8_t lcd_buffer[13] = {0};
	sprintf((char *) lcd_buffer, "%f", ms);   

	/* Display the contents of lcd_buffer onto the debug LCD */
    //DisplayLCD(LCD_LINE5, lcd_buffer);
		
	
	/* Use CMT channel 0 for a 1ms pause */
	err &=  R_CMT_CreateOneShot(
								0,
								0,
								ms,
								PDL_NO_FUNC,
								0);
	while (!err);
	
	if (err == 1)
		return 0;
	else
		return -1;
}


//get_ms
int get_ms(unsigned long *count)
{
	extern volatile uint32_t gTimerCount;
	
    if (!count)
        return 1;
    count[0] = gTimerCount;

    return 0;
}


