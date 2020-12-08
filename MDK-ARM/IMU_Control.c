#include <stdio.h>
#include "main.h"
#include "IMU_Control.h"
#include "tm_stm32_ahrs_imu.h"

// Defines
#define PI 3.14159265359
// Accelerometer registers
#define Addr_Accl 0x18		// I2C address
#define BMX055_ACC_WHOAMI        0x00  // should return 0xFA
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
// Acc. Compensation registers
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
// BMX055 Gyroscope Registers
#define Addr_Gyro 0x68		// I2C address
#define BMX055_GYRO_WHOAMI       0x00  // should return 0x0F
#define BMX055_GYRO_RANGE        0x0F
#define BMX055_GYRO_BW           0x10
#define BMX055_GYRO_LPM1         0x11

// Variables
float xAccl, yAccl, zAccl, xGyro, yGyro, zGyro;
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float Ares, Gres;
uint8_t acc[6], gyr[6];
uint8_t c,d,e;
float Ares = 2.0/2048.0;
float Gres = 124.87/32768.0;

//Functions

// BMX055 Setup function
void BMX_Setup()
{
	//Accel Setup
	writeByte(Addr_Accl, BMX055_ACC_PMU_RANGE, 0x03); // Range 2G
	writeByte(Addr_Accl, BMX055_ACC_PMU_BW, 0x08); // Bandwith 7.81 Hz
	writeByte(Addr_Accl, BMX055_ACC_PMU_LPW, 0x00); // Normal mode
	//Gyro Setup
	writeByte(Addr_Gyro, BMX055_GYRO_RANGE, 0x04); 	// 125 DPS
	writeByte(Addr_Gyro, BMX055_GYRO_BW, 0x07); 		// 100 Hz
	writeByte(Addr_Gyro, BMX055_GYRO_LPM1, 0x00); 	// Normal mode
	HAL_Delay(300); // Wait
}

// Read accelerometer and gyro data function
void read_acc_gyro()
{
	// Accel read and convert
		for (int i = 0; i < 6; i++)
			acc[i] = readByte(Addr_Accl, (2+i));

		xAccl = ((acc[1] * 256) + (acc[0] & 0xF0)) / 16;
		if (xAccl > 2047)
			xAccl -= 4096;
		
		yAccl = ((acc[3] * 256) + (acc[2] & 0xF0)) / 16;
		if (yAccl > 2047)
			yAccl -= 4096;
		
		zAccl = ((acc[5] * 256) + (acc[4] & 0xF0)) / 16;
		if (zAccl > 2047)
			zAccl -= 4096;
		
		// Gyro read and convert
		for (int i = 0; i < 6; i++)
			gyr[i] = readByte(Addr_Gyro, (2+i));

		xGyro = (gyr[1] * 256) + gyr[0];
		if (xGyro > 32767)
			xGyro -= 65536;
		
		yGyro = (gyr[3] * 256) + gyr[2];
		if (yGyro > 32767)
			yGyro -= 65536;
		
		zGyro = (gyr[5] * 256) + gyr[4];
		if (zGyro > 32767)
			zGyro -= 65536;
	
		
		//Conversion plus compensation
		xAccl = xAccl*Ares+accelBias[0];
		yAccl = yAccl*Ares+accelBias[1];
		zAccl = zAccl*Ares+accelBias[2];
		
		//Limits
		if(xAccl>1)
				xAccl=1;
			else if (xAccl<-1)
				xAccl=-1;
			if(yAccl>1)
				yAccl=1;
			else if (yAccl<-1)
				yAccl=-1;
			if(zAccl>1)
				zAccl=1;
			else if (zAccl<-1)
				zAccl=-1;
		
		// Degree to rad/s conversion for fusion library
		xGyro = AHRSIMU_DEG2RAD(xGyro*Gres);
		yGyro = AHRSIMU_DEG2RAD(yGyro*Gres);
		zGyro = AHRSIMU_DEG2RAD(zGyro*Gres);
	
}

// Fast accelerometer compensation function
void fastcompaccelBMX055() 
{
  writeByte(Addr_Accl, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
  writeByte(Addr_Accl, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
  writeByte(Addr_Accl, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}
  writeByte(Addr_Accl, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}
  writeByte(Addr_Accl, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(Addr_Accl, BMX055_ACC_OFC_CTRL);
  HAL_Delay(10);
}

  int8_t compx = readByte(Addr_Accl, BMX055_ACC_OFC_OFFSET_X);
  int8_t compy = readByte(Addr_Accl, BMX055_ACC_OFC_OFFSET_Y);
  int8_t compz = readByte(Addr_Accl, BMX055_ACC_OFC_OFFSET_Z);

  accelBias[0] = (float) compx/128.; // accleration bias in g
  accelBias[1] = (float) compy/128.; // accleration bias in g
  accelBias[2] = (float) compz/128.; // accleration bias in g
}
