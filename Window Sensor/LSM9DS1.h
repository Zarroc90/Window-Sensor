/*
 * LSM9DS1.h
 *
 *  Created on: 25.08.2016
 *      Author: Marvin
 */

#ifndef LSM9DS1_H_
#define LSM9DS1_H_

/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP     0x68
#define WHO_AM_I_M_RSP      0x3D

#define LSM9DS1_AG			CS_0
#define LSM9DS1_M			CS_1

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
	A_SCALE_2G,	// 00:  2g
	A_SCALE_16G,// 01:  16g
	A_SCALE_4G,	// 10:  4g
	A_SCALE_8G	// 11:  8g
};

// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum gyro_scale
{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS=3,	// 11:  2000 dps
};

// mag_scale defines all possible FSR's of the magnetometer:
enum mag_scale
{
	M_SCALE_4GS, 	// 00:  4Gs
	M_SCALE_8GS,	// 01:  8Gs
	M_SCALE_12GS,	// 10:  12Gs
	M_SCALE_16GS,	// 11:  16Gs
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum gyro_odr
{
	//! TODO
	G_ODR_PD,	// Power down (0)
	G_ODR_149,	// 14.9 Hz (1)
	G_ODR_595,	// 59.5 Hz (2)
	G_ODR_119,	// 119 Hz (3)
	G_ODR_238,	// 238 Hz (4)
	G_ODR_476,	// 476 Hz (5)
	G_ODR_952	// 952 Hz (6)
};
// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
	XL_POWER_DOWN, 	// Power-down mode (0x0)
	XL_ODR_10,		// 10 Hz (0x1)
	XL_ODR_50,		// 50 Hz (0x02)
	XL_ODR_119,		// 119 Hz (0x3)
	XL_ODR_238,		// 238 Hz (0x4)
	XL_ODR_476,		// 476 Hz (0x5)
	XL_ODR_952		// 952 Hz (0x6)
};

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
	A_ABW_408,		// 408 Hz (0x0)
	A_ABW_211,		// 211 Hz (0x1)
	A_ABW_105,		// 105 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
};


// mag_odr defines all possible output data rates of the magnetometer:
enum mag_odr
{
	M_ODR_0625,	// 0.625 Hz (0)
	M_ODR_125,	// 1.25 Hz (1)
	M_ODR_250,	// 2.5 Hz (2)
	M_ODR_5,	// 5 Hz (3)
	M_ODR_10,	// 10 Hz (4)
	M_ODR_20,	// 20 Hz (5)
	M_ODR_40,	// 40 Hz (6)
	M_ODR_80	// 80 Hz (7)
};

enum interrupt_select
{
	XG_INT1 = INT1_CTRL,
	XG_INT2 = INT2_CTRL
};

enum interrupt_generators
{
	INT_DRDY_XL = (1<<0),	// Valid for INT1 & INT2
	INT_DRDY_G = (1<<1),	// Valid for INT1 & INT2
	INT2_DRDY_TEMP = (1<<2),// Replaces INT_BOOT (int1)
	INT_FTH = (1<<3),		// Valid for INT1 & INT2
	INT_OVR = (1<<4),		// Valid for INT1 & INT2
	INT_FSS5 = (1<<5),		// Valid for INT1 & INT2
	INT1_IG_XL = (1<<6),	// ONLY VALID for INT1
	INT1_IG_G = (1<<7)		// Replaces INT2_INACT (int2)
};

enum accel_interrupt_generator
{
	XLIE_XL = (1<<0),
	XHIE_XL = (1<<1),
	YLIE_XL = (1<<2),
	YHIE_XL = (1<<3),
	ZLIE_XL = (1<<4),
	ZHIE_XL = (1<<5),
	GEN_6D = (1<<6)
};

enum gyro_interrupt_generator
{
	XLIE_G = (1<<0),
	XHIE_G = (1<<1),
	YLIE_G = (1<<2),
	YHIE_G = (1<<3),
	ZLIE_G = (1<<4),
	ZHIE_G = (1<<5)
};

enum mag_interrupt_generator
{
	ZIEN = (1<<5),
	YIEN = (1<<6),
	XIEN = (1<<7)
};

enum h_lactive
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
};

enum pp_od
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
};

enum fifoMode_type
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 5
};

struct gyroSettings
{
	// Gyroscope settings:
	int enabled;
	int scale;	// Changed this to 16-bit
	int sampleRate;
	// New gyro stuff:
	int bandwidth;
	int lowPowerEnable;
	int HPFEnable;
	int HPFCutoff;
	int flipX;
	int flipY;
	int flipZ;
	int orientation;
	int enableX;
	int enableY;
	int enableZ;
	int latchInterrupt;
};

struct deviceSettings
{
    int commInterface; // Can be I2C, SPI 4-wire or SPI 3-wire
    int agAddress;	// I2C address or SPI CS pin
	int mAddress;	// I2C address or SPI CS pin
};

struct accelSettings
{
	// Accelerometer settings:
    int enabled;
    int scale;
    int sampleRate;
	// New accel stuff:
	int enableX;
	int enableY;
	int enableZ;
	int bandwidth;
	int highResEnable;
	int highResBandwidth;
};

struct magSettings
{
	// Magnetometer settings:
    int enabled;
    int scale;
    int sampleRate;
	// New mag stuff:
	int tempCompensationEnable;
	int XYPerformance;
	int ZPerformance;
	int lowPowerEnable;
	int operatingMode;
};

struct temperatureSettings
{
	// Temperature settings
    int enabled;
};

/*struct IMUSettings
{
	deviceSettings device;

	gyroSettings gyro;
	accelSettings accel;
	magSettings mag;

	temperatureSettings temp;
};*/


#endif /* LSM9DS1_H_ */
