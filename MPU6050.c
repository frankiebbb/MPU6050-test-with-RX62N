#include <stdio.h>
#include "lcd.h"
#include "MPU6050.h"

/* RPDL definitions */
#include "r_pdl_iic.h"

/* RPDL device-specific definitions */
#include "r_pdl_definitions.h"


#define gyro_xsensitivity 66.5 //66.5 Dead on at last check
#define gyro_ysensitivity 66.5 //72.7 Dead on at last check
#define gyro_zsensitivity 65.5
#define a 0.01

volatile uint8_t data_array[5];

 
void Setup_MPU6050()
{
	unsigned char error;
	//Sets sample rate to 1000/1+1 = 500Hz
	error = ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01);
	if(error ==1) {
    	lcd_line_four();
    	lcd_write_string(PSTR("ByteWrite Fail"));
    }
	else if (error == 0) {
    	lcd_line_four();
    	lcd_write_string(PSTR("ByteWrite Pass"));
    }
	lcd_line_four();
	lcd_write_string(PSTR("ByteWrite"));
	//Disable FSync, 48Hz DLPF
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x03);
	//Disable gyro self tests, scale of 500 degrees/s
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0b00001000);
	//Disable accel self tests, scale of +-4g, no DHPF
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0b00001000);
	//Freefall threshold of <|0mg|
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 0x00);
	//Motion threshold of >0mg
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 0x00);
	//Motion duration of >0s
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 0x00);
	//Zero motion threshold
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 0x00);
	//Disable sensor output to FIFO buffer
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);
 
	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_REG, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
 	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_REG, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_REG, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_REG, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_REG, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DO, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
  	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV4_DI, 0x00);
 
  	//MPU6050_RA_I2C_MST_STATUS //Read-only
  	//Setup INT pin and AUX I2C pass through
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x00);
	//Enable data ready interrupt
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);
 
	//MPU6050_RA_DMP_INT_STATUS //Read-only
	//MPU6050_RA_INT_STATUS 3A //Read-only
    //MPU6050_RA_ACCEL_XOUT_H //Read-only
    //MPU6050_RA_ACCEL_XOUT_L //Read-only
    //MPU6050_RA_ACCEL_YOUT_H //Read-only
    //MPU6050_RA_ACCEL_YOUT_L //Read-only
    //MPU6050_RA_ACCEL_ZOUT_H //Read-only
    //MPU6050_RA_ACCEL_ZOUT_L //Read-only
    //MPU6050_RA_TEMP_OUT_H //Read-only
    //MPU6050_RA_TEMP_OUT_L //Read-only
    //MPU6050_RA_GYRO_XOUT_H //Read-only
    //MPU6050_RA_GYRO_XOUT_L //Read-only
    //MPU6050_RA_GYRO_YOUT_H //Read-only
    //MPU6050_RA_GYRO_YOUT_L //Read-only
    //MPU6050_RA_GYRO_ZOUT_H //Read-only
    //MPU6050_RA_GYRO_ZOUT_L //Read-only
    //MPU6050_RA_EXT_SENS_DATA_00 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_01 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_02 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_03 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_04 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_05 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_06 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_07 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_08 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_09 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_10 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_11 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_12 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_13 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_14 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_15 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_16 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_17 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_18 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_19 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_20 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_21 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_22 //Read-only
    //MPU6050_RA_EXT_SENS_DATA_23 //Read-only
    //MPU6050_RA_MOT_DETECT_STATUS //Read-only
 
	//Slave out, dont care
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);
	//More slave config
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	//Motion detection control
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
	//Sets clock source to gyro reference w/ PLL
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
    //MPU6050_RA_BANK_SEL //Not in datasheet
    //MPU6050_RA_MEM_START_ADDR //Not in datasheet
    //MPU6050_RA_MEM_R_W //Not in datasheet
    //MPU6050_RA_DMP_CFG_1 //Not in datasheet
    //MPU6050_RA_DMP_CFG_2 //Not in datasheet
    //MPU6050_RA_FIFO_COUNTH //Read-only
    //MPU6050_RA_FIFO_COUNTL //Read-only
	//Data transfer to and from the FIFO buffer
	ByteWriteI2C(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);
    //MPU6050_RA_WHO_AM_I //Read-only, I2C address
 
	lcd_home();
	lcd_write_string(PSTR("MPU6050 Setup Done"));

}
 
int MPU6050_Test_I2C()
{
	unsigned char Data = 0x00;
	ByteReadI2C(MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, Data);
 
	if(Data == 0x68)
	{
		lcd_line_two();
		lcd_write_string(PSTR("Read Test Passed"));
		//printf("\nI2C Read Test Passed, MPU6050 Address: 0x%x", Data);
		return 0;
	}
	else
	{
		lcd_line_two();
		lcd_write_string(PSTR("Read Test Failed"));
		//printf("ERROR: I2C Read Test Failed, Stopping");
		//while(1){}
		return 1;
	}
}

