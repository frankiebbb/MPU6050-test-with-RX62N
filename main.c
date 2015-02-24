/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*******************************************************************************/
/*******************************************************************************
* Copyright (C) 2010 Renesas Electronics Corporation. All rights reserved.    */ 
/*******************************************************************************
* File Name    : main.c
* Version      : 1.00
* Device       : R5F562N8
* Tool Chain   : RX Family C Compiler
* H/W Platform : YRDKRX62N
* Description  : Empty application project.
*******************************************************************************/
/*******************************************************************************
* History	   : 27.08.2010 Ver. 1.00 First Release
*******************************************************************************/

/*******************************************************************************
* Outline      : main
* Description  : Main program function
* Argument     : none
* Return value : none
*******************************************************************************/
/*void main(void)
{
	while (1) ;	
}*/
/*******************************************************************************
* End of function main
*******************************************************************************/


/*******************************************************************************
* Project Includes
*******************************************************************************/    
#include <stdbool.h>

/* Defines LCD functions used in this file */
#include "lcd.h"

/* Defines IIC parameters */
#include"iic_defines.h"

/* IIC RPDL function definitions */
#include "r_pdl_iic.h"

/* RPDL definitions */
#include "r_pdl_intc.h"

/* IO Port RPDL function definitions */
#include "r_pdl_io_port.h"

/* General RPDL function definitions */
#include "r_pdl_definitions.h"

/* RSPI API library support */
#include "YRDKRX62N_RSPI_API.h"

/* Board specific definitions */
#include "YRDKRX62N.h"

/* Defines timer functions used in this file */
#include "timer_compare.h"

/* MPU6050 definitions */
#include "MPU6050.h"

/* Switch handler function definitions */
#include "switch.h"

#include "app_lib.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* External functions */
//void Init_Thermal_Sensor( void );
//void Init_Accelerometer( void );

#include <string.h>
#include <stdio.h>
#include <math.h>


#define MOTION_DRIVER_TARGET_RENESAS


/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define MPU6050

#define QUAT_SENS		(1073741824.0)
#define PI				3.1415926536ACCEL_SENS = 16384.0GYRO_SENS  = 16.375QUAT_SENS  = 1073741824.0ACCEL_SENS = 16384.0GYRO_SENS  = 16.375QUAT_SENS  = 1073741824.0



struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};


unsigned long sensor_timestamp = 0;

double old_yaw = 0.0;
double drift_rate = 0.0;
double old_drift_rate = 0;
unsigned int count = 0;
unsigned long old_sensor_timestamp = 0;

double dif_yaw;

int samples = 32;
double offset[3] = {0};
double acc_avg[3] = {0};
float old_acc[3] = {0};

//double distance[3] = {0};

bool sw1 = false;
bool sw2 = false;

	int x = 0;
	int z = 0;
	double GYRO_OFFSET[3] = {0};
	double GYRO_XOUT_OFFSET_1000SUM = 0;
	double GYRO_YOUT_OFFSET_1000SUM = 0;
	double GYRO_ZOUT_OFFSET_1000SUM = 0;
	
	
	
	
    /* Kalman filter variables */
    /* We will set the variables like so, these can also be tuned by the user */
    double Q_angle = 0.001; // Process noise variance for the accelerometer
    double Q_bias = 0.03;	// Process noise variance for the gyro bias
    double R_measure = 0.01; /*Measurement noise variance - this is actually the variance of the 
						measurement noise*/

    double angle = 0; 	// The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias = 0;	// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; 	/* Unbiased rate calculated from the rate and the calculated bias - 
					   you have to call getAngle to update the rate*/

    double P[2][2] = {0}; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error		
	
	
	float yaw_offset = 0;
	float yaw_sum = 0 ;
	float time = 0;

	float vel = 0;
	float pos = 0;;
	float old_vel = 0;
	float old_pos = 0;
	
	float grav[3] = {0};
	
	float y_ang = 0;
	float old_y = 0;


	//float accel_sum[3] = {0};
	//float accel_offset[3] = {0};
	
	float accel_max = 0;
	float accel_min = 0;

	float o_yaw = 0;
	bool first_yaw = 0;
	float yaw_ref = 0;

	
	
	
/* Local functions */
void Init_IIC_Master(void);


static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    //send_packet(PACKET_TYPE_TAP, data);
}


static inline void run_self_test(void)
{
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;
	
    result = mpu_run_self_test(gyro, accel);

    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 4096.f; //convert to +-8G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        //mpu_set_gyro_bias_reg(gyro);
        //mpu_set_accel_bias_6050_reg(accel);
		
		dmp_set_gyro_bias(gyro);
		dmp_set_accel_bias(accel);
    }
}


/*******************************************************************************
* Outline       : CB_SwitchPress
* Description   : Switch press callback function. This function resets the value
*                 of gTimerCount, and starts the CMT timer if switch SW1 has
*                 been pressed.
* Argument      : none
* Return value  : none
*******************************************************************************/
void CB_SwitchPress(void)
{	
	ClearLCD();
    DisplayLCD(LCD_LINE1, "MPU6050 Test");
	/* Check if switch SW1 has been pressed */
  	if (gSwitchFlag & SWITCHHOLD_1)
  	{
    	hal.report = PRINT_ACCEL;
		
		sw1 = !sw1;
		if (sw1){
			vel = 0;	
			pos = 0;
		}
		else{
			
		}	
  	}
  	else if (gSwitchFlag & SWITCHHOLD_2)
  	{
    	hal.report = PRINT_GYRO;
		
		sw2 = !sw2;
		
		if (sw2){
			y_ang = 0;	
			old_y = 0;
		}
  	}
  	else if (gSwitchFlag & SWITCHHOLD_3)
  	{
    	hal.report = PRINT_QUAT;
  	}
}
/*******************************************************************************
* End of function CB_SwitchPress
*******************************************************************************/

double getAngle(double newAngle, double newRate, double dt) 
{
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    S = P[0][0] + R_measure;
    
	/* Step 5 */
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    y = newAngle - angle;
    
	/* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return angle;
}


void CallBackFunc( void ){
	uint8_t lcd_buffer[13] = {0};
	unsigned char tmp1;

    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
	
	float temp[4];
	float roll, pitch, yaw, yaw_deg;

	float gyroZangle;	// Angle calculate using the gyro only
	float compAngleZ; 	// Calculated angle using a complementary filter
	float kalAngleZ; 	// Calculated angle using a Kalman filter
	

	
	
    /* This function gets new data from the FIFO when the DMP is in
     * use. The FIFO can contain any combination of gyro, accel,
     * quaternion, and gesture data. The sensors parameter tells the
     * caller which data fields were actually populated with new data.
     * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
     * the FIFO isn't being filled with accel data.
     * The driver parses the gesture data to determine if a gesture
     * event has occurred; on an event, the application will be notified
     * via a callback (assuming that a callback function was properly
     * registered). The more parameter is non-zero if there are
     * leftover packets in the FIFO.
     */
	 
	do{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
	        &more);
	    if (!more)
	        hal.new_gyro = 0;
	    /* Gyro and accel data are written to the FIFO by the DMP in chip
	     * frame and hardware units. This behavior is convenient because it
	     * keeps the gyro and accel outputs of dmp_read_fifo and
	     * mpu_read_fifo consistent.
	     */
	    if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO){
			// Calculate delta time
		  	double dt = (double)(sensor_timestamp - old_sensor_timestamp) / 10000;
			
			if (dt < 0){
				dt = (double)(0xffffffff - old_sensor_timestamp + sensor_timestamp) / 10000;
			}
			
			for (int i = 0; i<3; i++){
				temp[i] = (gyro[i] - GYRO_OFFSET[i]) / 16.384 * 4;
				
				if (temp[i] >= -0.1 && temp[i] <= 0.1){
					temp[i] = 0;
				}

				sprintf((char *) lcd_buffer, "g%d: %5.2f", i, temp[i]);   
				/* Display the contents of lcd_buffer onto the debug LCD */
				DisplayLCD(LCD_LINE2 + 8*i, lcd_buffer);
			}

			if (sw2){
				//Rutina para encontrar el bias del gyro
				GYRO_XOUT_OFFSET_1000SUM += gyro[0];
				GYRO_YOUT_OFFSET_1000SUM += gyro[1];
				GYRO_ZOUT_OFFSET_1000SUM += gyro[2];
		
				x++;
				
				if (x == 500){
					x = 0;
					sw2 = !sw2;
					R_IO_PORT_Modify(LED4, PDL_IO_PORT_XOR, 1);	
				 	
					GYRO_OFFSET[0] = GYRO_XOUT_OFFSET_1000SUM/500;
					GYRO_OFFSET[1] = GYRO_YOUT_OFFSET_1000SUM/500;
					GYRO_OFFSET[2] = GYRO_ZOUT_OFFSET_1000SUM/500;
				
					GYRO_XOUT_OFFSET_1000SUM = 0;
					GYRO_YOUT_OFFSET_1000SUM = 0;
					GYRO_ZOUT_OFFSET_1000SUM = 0;
				}
			}
			
			//temp[2] = (gyro[2] - GYRO_OFFSET[2]) / 16.384;
			
			if (temp[2] >= -0.1 && temp[2] <= 0.1)
				temp[2] = 0;

			y_ang += (temp[2] + old_y) / 2 * dt;  
			
			
			sprintf((char *) lcd_buffer, "%4.2f", y_ang);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE5, lcd_buffer);
			
			old_sensor_timestamp = sensor_timestamp;
			old_y = temp[2];

		}
		 
		 
//Accelerometer
	    if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL){

		  	float dt = (float)(sensor_timestamp - old_sensor_timestamp) / 10000;
			old_sensor_timestamp = sensor_timestamp;

			sprintf((char *) lcd_buffer, "t: %1.2f", dt * 1000);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE5, lcd_buffer);


			for (int i = 0; i<4; i++){
				temp[i] = quat[i] / 1073741824.0;
			}
			
#if 1	
			/* Find Gravity*/
			grav[0] = 2 * (temp[1] * temp[3] - temp[0] * temp[2]);
			grav[1] = 2 * (temp[0] * temp[1] + temp[2] * temp[3]);
			grav[2] = (temp[0] * temp[0] - temp[1] * temp[1] - temp[2] * temp[2] 
						+ temp[3] * temp[3]);
			

			for (int i = 0; i<3; i++){
				temp[i] = (accel[i] / 16384.0 + grav[i]);
			}
			
			temp[0] = temp[0] - 0.01;
			
			if (temp[0] <= 0.03 && temp[0] >= -0.03){
				temp[0] = 0;	
			}	
			
			if (temp[0] > accel_max){
				accel_max = temp[0];
				
				sprintf((char *) lcd_buffer, "max: %2.5f", accel_max);   
				/* Display the contents of lcd_buffer onto the debug LCD */
				DisplayLCD(LCD_LINE7, lcd_buffer);
			}
			
			if (temp[0] < accel_min){
				accel_min = temp[0];

				sprintf((char *) lcd_buffer, "min: %2.5f", accel_min);   
				/* Display the contents of lcd_buffer onto the debug LCD */
				DisplayLCD(LCD_LINE8, lcd_buffer);
			}
					
				
			for (int i = 0; i<1; i++){
				sprintf((char *) lcd_buffer, "a: %7.3f", temp[i]);   
				/* Display the contents of lcd_buffer onto the debug LCD */
				DisplayLCD(LCD_LINE2 + 8*i, lcd_buffer);
			}				

			//first X integration:
			//vel= old_vel + old_acc[0] + ((temp[0] - old_acc[0]) / 2);
			vel= old_vel + (temp[0] * 9.80665 + old_acc[0]) / 2 * dt;
			
			
			if (temp[0] == 0){ 
				count++;
			}
			else { 
				count = 0;
			}
			
			//if this number exceeds 25, we can assume that velocity is cero
			if (count >= 20)          
			{ 
				vel = 0;
			}

			sprintf((char *) lcd_buffer, "v: %7.3f", vel * 3.6);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE3, lcd_buffer);
			
			sprintf((char *) lcd_buffer, "d: %7.3f", pos);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE4, lcd_buffer);			

			//second X integration:
			pos = old_pos + old_vel + ((vel - old_vel) / 2);


			//vel += (temp[0] * 9.80665) * dt;
			
			old_acc[0] = temp[0];
			old_vel = vel;
			old_pos = pos;
			
			if (sw1){
				accel_max = 0;
				accel_min = 0;
				sw1 = !sw1;
			}

#endif
			
#if 0
			/* Find Gravity*/
			grav[0] = 2 * (temp[1] * temp[3] - temp[0] * temp[2]);
			grav[1] = 2 * (temp[0] * temp[1] + temp[2] * temp[3]);
			grav[2] = (temp[0] * temp[0] - temp[1] * temp[1] - temp[2] * temp[2] 
						+ temp[3] * temp[3]);

			for (int i = 0; i<3; i++){
				temp[i] = accel[i] / 8192.0;
				grav[2] = - grav[2] - 0.08;
				sprintf((char *) lcd_buffer, "a%d: %7.2f", i, temp[i] + grav[i]);   
				/* Display the contents of lcd_buffer onto the debug LCD */
				DisplayLCD(LCD_LINE2 + 8*i, lcd_buffer);
			}
#endif
		}
			//send_packet(PACKET_TYPE_ACCEL, accel);
	    /* Unlike gyro and accel, quaternions are written to the FIFO in
	     * the body frame, q30. The orientation is set by the scalar passed
	     * to dmp_set_orientation during initialization.
	     */
	    if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT){
			//send_packet(PACKET_TYPE_QUAT, quat);
			//double norm;
			//norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
		
		
			for (int i = 0; i<4; i++){
				temp[i] = quat[i] / 1073741824.0;
			}
			roll = atan2(2*temp[0]*temp[1] + 2*temp[2]*temp[3], 
						1 - 2*temp[1]*temp[1] - 2*temp[2]*temp[2]);
						
			pitch = asin(2*temp[0]*temp[2] - 2*temp[3]*temp[1]);
										
			yaw = atan2(2*temp[0]*temp[3] + 2*temp[1]*temp[2], 
						1 - 2*temp[2]*temp[2] - 2*temp[3]*temp[3]);
				

			/*if(first_yaw){
				if((yaw - o_yaw) > 0  && (yaw - o_yaw) > 0.01){
					yaw_ref += yaw - o_yaw;
				}
				else{
					o_yaw = yaw;
				}

				if((yaw - o_yaw) < 0  && (o_yaw - yaw) > 0.01){
					yaw_ref += yaw - o_yaw;
				}
				else{
					o_yaw = yaw;
				}


			}
			
			while(!first_yaw){
				o_yaw = yaw;
				first_yaw = 1;
			}*/
			
	
			
				
			sprintf((char *) lcd_buffer, "r: %7.2f", roll*180.0/3.1415926536);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE6, lcd_buffer);
			sprintf((char *) lcd_buffer, "p: %7.2f", pitch*180.0/3.1415926536);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE7, lcd_buffer);
			sprintf((char *) lcd_buffer, "y: %7.2f", yaw*180.0/3.1415926536);   
			/* Display the contents of lcd_buffer onto the debug LCD */
			DisplayLCD(LCD_LINE8, lcd_buffer);
		}
	} while(more);	
}



void main(void)
{	
	//bool		err = true;
	//uint8_t  	target_reg; 
	//uint8_t 	target_data = 0;
	//uint8_t		slaveAddress;
    //uint16_t i;    
    uint8_t  	target_reg_data[4] = {0};
	//uint8_t		*regAddress;
	//const uint8_t target_reg_data[3] = {0x68, 0x19, 0x3f};
	/* Declare display buffer */
    uint8_t lcd_buffer[13] = {0};
	extern volatile uint32_t gTimerCount;

    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    //struct int_param_s int_param;
	
	/* Initialise the debug LCD on the RSPI bus */
    YRDKRX62N_RSPI_Init(RSPI_CHANNEL_0);
    InitialiseLCD();

    /* Display IIC mode on the debug LCD */
    DisplayLCD(LCD_LINE1, "MPU6050 Test");
    //DisplayLCD(LCD_LINE2, "    Test");
	
	//R_IO_PORT_Write( LED8, LED_OFF );
	
	/* Initialize IIC master */
    Init_IIC_Master();
	

	
	
	/* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    /*int_param.cb = gyro_data_ready_cb;
    int_param.pin = INT_PIN_P20;
    int_param.lp_exit = INT_EXIT_LPM0;
    int_param.active_low = 1;*/	
	result = mpu_init(/*&int_param*/);
	if (result)
		while(1);
		
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    //hal.report = PRINT_ACCEL;
    hal.report = PRINT_GYRO;
    //hal.report = PRINT_QUAT;
		
    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
    //dmp_register_tap_cb(tap_cb);
    //dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT /*| DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT*/ | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
	hal.new_gyro = 1;
	
    /* Configure the CMT timer for compare match operation */
    Init_CompareMatchTimer();
	
	/* Configure the callback function from the switch press interrupt */
  	SetSwitchPressCallback(CB_SwitchPress);

    //DisplayLCD(LCD_LINE3, "     OK");	
	
	unsigned char tmp1;	
	
	run_self_test();

	//while(dmp_enable_gyro_cal(1));
    
	/* Declare error flag */
    bool err = true;

	/* Configure the IRQ11 interrupt */
	err &= R_INTC_CreateExtInterrupt(	PDL_INTC_IRQ11,
										PDL_INTC_FALLING|PDL_INTC_B,
										CallBackFunc,
										5);
										
	while (!err);
	
	R_IO_PORT_Modify(LED13, PDL_IO_PORT_XOR, 1);
	
	while(1){
	}
}


/*******************************************************************************
* Outline      : Init_IIC_Master
* Description  : This function configures the IIC unit to operate in bus master
*                 mode. 
* Argument     : none
* Return value : none
*******************************************************************************/
void Init_IIC_Master(void)
{
    /* Declare error flag */
    bool err = true;

    /* Configure the IIC unit to operate as bus master */
    err &=    R_IIC_Create
              (
              IIC_CHANNEL,
              PDL_IIC_MODE_IIC|PDL_IIC_INT_PCLK_DIV_8,
              //PDL_IIC_SLAVE_0_ENABLE_7,
              //0xD0,
			  PDL_NO_DATA,
              PDL_NO_DATA,
              PDL_NO_DATA,
              PDL_NO_DATA,
              IIC_TRANSFER_DATA_RATE,
              (SCL_RISE_TIME << 16) | SCL_FALL_TIME
              );            

    /* Halt in while loop when RPDL errors detected */    
    while (!err);
}
/*******************************************************************************
* End of function Init_IIC_Master
*******************************************************************************/


