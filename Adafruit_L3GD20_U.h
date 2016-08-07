/***************************************************
  This is a library for the L3GD20 GYROSCOPE

  Designed specifically to work with the Adafruit L3GD20 Breakout
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C)
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef __L3GD20_H__
#define __L3GD20_H__

#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

 /*=========================================================================
	 I2C ADDRESS/BITS AND SETTINGS
	 -----------------------------------------------------------------------*/
#define L3GD20_ADDRESS           (0x6B)        // 1101011
#define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
#define L3GD20_ID                0xD4
#define L3GD20H_ID               0xD7
#define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256
	 /*=========================================================================*/

	 /*=========================================================================
		 REGISTERS
		 -----------------------------------------------------------------------*/
typedef enum
{                                             // DEFAULT    TYPE
	GYRO_REGISTER_WHO_AM_I = 0x0F,   // 11010100   r
	GYRO_REGISTER_CTRL_REG1 = 0x20,   // 00000111   rw
	GYRO_REGISTER_CTRL_REG2 = 0x21,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG3 = 0x22,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG4 = 0x23,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG5 = 0x24,   // 00000000   rw
	GYRO_REGISTER_REFERENCE = 0x25,   // 00000000   rw
	GYRO_REGISTER_OUT_TEMP = 0x26,   //            r
	GYRO_REGISTER_STATUS_REG = 0x27,   //            r
	GYRO_REGISTER_OUT_X_L = 0x28,   //            r
	GYRO_REGISTER_OUT_X_H = 0x29,   //            r
	GYRO_REGISTER_OUT_Y_L = 0x2A,   //            r
	GYRO_REGISTER_OUT_Y_H = 0x2B,   //            r
	GYRO_REGISTER_OUT_Z_L = 0x2C,   //            r
	GYRO_REGISTER_OUT_Z_H = 0x2D,   //            r
	GYRO_REGISTER_FIFO_CTRL_REG = 0x2E,   // 00000000   rw
	GYRO_REGISTER_FIFO_SRC_REG = 0x2F,   //            r
	GYRO_REGISTER_INT1_CFG = 0x30,   // 00000000   rw
	GYRO_REGISTER_INT1_SRC = 0x31,   //            r
	GYRO_REGISTER_TSH_XH = 0x32,   // 00000000   rw
	GYRO_REGISTER_TSH_XL = 0x33,   // 00000000   rw
	GYRO_REGISTER_TSH_YH = 0x34,   // 00000000   rw
	GYRO_REGISTER_TSH_YL = 0x35,   // 00000000   rw
	GYRO_REGISTER_TSH_ZH = 0x36,   // 00000000   rw
	GYRO_REGISTER_TSH_ZL = 0x37,   // 00000000   rw
	GYRO_REGISTER_INT1_DURATION = 0x38    // 00000000   rw
} gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
	OPTIONAL SPEED SETTINGS
	-----------------------------------------------------------------------*/
typedef enum
{
	GYRO_RANGE_250DPS = 250,
	GYRO_RANGE_500DPS = 500,
	GYRO_RANGE_2000DPS = 2000
} gyroRange_t;
/*=========================================================================*/

/*=========================================================================
	OUTPUT DATA RATE SETTINGS
	-----------------------------------------------------------------------*/
typedef enum
{
	GYRO_ODR_95 = 0x00, // 95 Hz
	GYRO_ODR_190 = 0x01, // 190 Hz
	GYRO_ODR_380 = 0x02, // 380 Hz
	GYRO_ODR_760 = 0x03 // 760 Hz
} gyroDataRate;
/*=========================================================================*/

class Adafruit_L3GD20_Unified
{
public:
	Adafruit_L3GD20_Unified(TwoWire* wire, int32_t sensorID = -1) :
		_wire(wire),
		_range(),
		_sensorID(sensorID),
		_autoRangeEnabled(false) {}
	Adafruit_L3GD20_Unified(int32_t sensorID = -1) :
		Adafruit_L3GD20_Unified(&Wire, sensorID) {}

	bool begin(gyroRange_t rng = GYRO_RANGE_250DPS);
	void enableAutoRange(bool enabled);
	bool enableDRDYInterrupt(bool enabled);
	void setOutputDataRate(gyroDataRate odr);
	bool getGyro(sensors_vec_t* gyro);

private:
	bool        write8(uint8_t reg, uint8_t value);
	bool        read8(uint8_t reg, uint8_t *value);
	TwoWire*    _wire;
	gyroRange_t _range;
	int32_t     _sensorID;
	bool        _autoRangeEnabled;
};

#endif
