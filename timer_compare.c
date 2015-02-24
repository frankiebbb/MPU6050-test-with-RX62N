/******************************************************************************
* DISCLAIMER

* This software is supplied by Renesas Electronics Corp. and is
* only intended for use with Renesas products.  

* No other uses are authorized.

* This software is owned by Renesas Electronics Corp. and is 
* protected under the applicable laws, including copyright laws.

* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES
* REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* INCLUDING BUT NOT LIMITED TO WAWRRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.  ALL SUCH
* WARRANTIES ARE EXPRESSLY DISCLAIMED.

* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER 
* RENESAS ELECTRONICS CORP. NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR AND DIRECT, INDIRECT, SPECIAL, INCIDENTAL
* OR COSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE,
* EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE
* POSSIBILITIES OF SUCH DAMAGES.

* Renesas reserves the right, without notice, to make changes to this
* software and to discontinue availability of this software.
* By using this software, you agree to the additional terms and 
* conditions found by accessing the following link:
* http://www.renesas.com/disclaimer
*******************************************************************************/
/* Copyright (C) 2010. Renesas Electronics Corp., All Rights Reserved         */
/******************************************************************************
* File Name     : timer_compare.c
* Version       : 1.1
* Device(s)     : R5F562N8
* Tool-Chain    : Renesas RX Standard Toolchain 1.0.1
* OS            : None
* H/W Platform  : YRDKRX62N
* Description   : Defines timer functions used to configure the CMT unit to 
*                 toggle the user LEDs on every compare match.
* Limitations   : None
*******************************************************************************
* History : DD.MMM.YYYY     Description
*         : 08.Oct.2010     First release
*         : 02.Dec.2010     Second YRDK release
*******************************************************************************/

/*******************************************************************************
* Project Includes
*******************************************************************************/
/* IO Port RPDL function definitions */
#include "r_pdl_io_port.h"
/* CMT RPDL function definitions */
#include "r_pdl_cmt.h"
/* General RPDL function definitions */
#include "r_pdl_definitions.h"
/* Provides declarations of functions defined in this file */
#include "timer_compare.h"

/* Board specific definitions */
#include "YRDKRX62N.h"

#include <stdio.h>
#include <string.h>
/* Defines LCD functions used in this file */
#include "lcd.h"


volatile uint32_t gTimerCount = 0;
volatile uint32_t tcount = 0;



/*******************************************************************************
* Local Function Prototypes
*******************************************************************************/
/* Compare match callback function declaration */
void CB_CompareMatch(void);

/*******************************************************************************
* Outline       : Init_CompareMatchTimer
* Description   : This function initialises a CMT timer that executes a callback
*                 function everytime a compare match occurs.
* Argument      : none
* Return value  : none
*******************************************************************************/
void Init_CompareMatchTimer(void)
{
    /* Declare error flag */
    bool err = true;

    /* CMT is configured for a 1ms interval, and executes the callback 
       function CB_CompareMatch on every compare match */
    err &=  R_CMT_Create(3,
						 PDL_CMT_FREQUENCY,
            			 10000,
            			 CB_CompareMatch,
            			 10);

    /* Halt in while loop when RPDL errors detected */  
    while (!err);
}
/*******************************************************************************
* End of function Init_CompareMatchTimer
*******************************************************************************/

/*******************************************************************************
* Outline       : CB_CompareMatch
* Description   : This callback function is called as an ISR whenever a compare
*                 match on CMT channel 3 occurs. The function toggles the state
*                 of the user LEDs.
* Argument      : none
* Return value  : none
*******************************************************************************/
void CB_CompareMatch(void)
{
	
    /* Declare error flag */
    //bool err = true;
	
	/* Declare display buffer */
    //uint8_t lcd_buffer[13] = {0};
	
	gTimerCount++;

	/*tcount++;
	if (tcount == 100000){
		R_IO_PORT_Modify(LED6, PDL_IO_PORT_XOR, 1);	
		tcount = 0;
	}*/

	
    //sprintf((char *) lcd_buffer, "  %u   " , (unsigned int *)gTimerCount);   

    /* Display the contents of lcd_buffer onto the debug LCD */
    //DisplayLCD(LCD_LINE4, lcd_buffer);

          
}
/*******************************************************************************
* End of function CB_CompareMatch
*******************************************************************************/
