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
#include "Arduino.h"
#include "Adafruit_L3GD20_U.h"

 /***************************************************************************
  PRIVATE FUNCTIONS
  ***************************************************************************/

#define write(reg, value) _wire->write(L3GD20_ADDRESS, reg, (uint8_t)value)
#define read8(reg, value) _wire->read(L3GD20_ADDRESS, reg, 1, value)

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
	 @brief  Setups the HW
 */
 /**************************************************************************/
bool Adafruit_L3GD20_Unified::begin(gyroRange_t rng)
{
	/* Enable I2C */
	_wire->begin();

	/* Set the range the an appropriate value */
	_range = rng;

	/* Make sure we have the correct chip ID since this checks
	   for correct address and that the IC is properly connected */
	uint8_t id;
	if (read8(GYRO_REGISTER_WHO_AM_I, &id) || ((id != L3GD20_ID) && (id != L3GD20H_ID)))
	{
		return false;
	}

	/* Set CTRL_REG1 (0x20)
	 ====================================================================
	 BIT  Symbol    Description                                   Default
	 ---  ------    --------------------------------------------- -------
	 7-6  DR1/0     Output data rate                                   00
	 5-4  BW1/0     Bandwidth selection                                00
	   3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
	   2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
	   1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
	   0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

	   /* Reset then switch to normal mode and enable all three channels */
	write(GYRO_REGISTER_CTRL_REG1, 0x00);
	write(GYRO_REGISTER_CTRL_REG1, 0x0F);
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG2 (0x21)
	 ====================================================================
	 BIT  Symbol    Description                                   Default
	 ---  ------    --------------------------------------------- -------
	 5-4  HPM1/0    High-pass filter mode selection                    00
	 3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

	 /* Nothing to do ... keep default values */
	 /* ------------------------------------------------------------------ */

	 /* Set CTRL_REG3 (0x22)
	  ====================================================================
	  BIT  Symbol    Description                                   Default
	  ---  ------    --------------------------------------------- -------
		7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
		6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
		5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
		4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
		3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
		2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
		1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
		0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

		/* Nothing to do ... keep default values */
		/* ------------------------------------------------------------------ */

		/* Set CTRL_REG4 (0x23)
		 ====================================================================
		 BIT  Symbol    Description                                   Default
		 ---  ------    --------------------------------------------- -------
		   7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
		   6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
		 5-4  FS1/0     Full scale selection                               00
										00 = 250 dps
										01 = 500 dps
										10 = 2000 dps
										11 = 2000 dps
		   0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

		   /* Adjust resolution if requested */
	switch (_range)
	{
	case GYRO_RANGE_250DPS:
		write(GYRO_REGISTER_CTRL_REG4, 0x00);
		break;
	case GYRO_RANGE_500DPS:
		write(GYRO_REGISTER_CTRL_REG4, 0x10);
		break;
	case GYRO_RANGE_2000DPS:
		write(GYRO_REGISTER_CTRL_REG4, 0x20);
		break;
	}
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG5 (0x24)
	 ====================================================================
	 BIT  Symbol    Description                                   Default
	 ---  ------    --------------------------------------------- -------
	   7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
	   6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
	   4  HPen      High-pass filter enable (0=disable,1=enable)        0
	 3-2  INT1_SEL  INT1 Selection config                              00
	 1-0  OUT_SEL   Out selection config                               00 */

	 /* Nothing to do ... keep default values */
	 /* ------------------------------------------------------------------ */

	return true;
}

/**************************************************************************/
/*!
	@brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void Adafruit_L3GD20_Unified::enableAutoRange(bool enabled)
{
	_autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
	@brief  Enables (or disables) the Data Ready interrupt
*/
/**************************************************************************/
bool Adafruit_L3GD20_Unified::enableDRDYInterrupt(bool enabled)
{
	uint8_t existing;
	if (read8(GYRO_REGISTER_CTRL_REG3, &existing))
	{
		return false;
	}

	if (enabled)
	{
		write(GYRO_REGISTER_CTRL_REG3, (existing |= 1 << 3));
	}
	else
	{
		write(GYRO_REGISTER_CTRL_REG3, (existing &= ~(1 << 3)));
	}
	return true;
}

/**************************************************************************/
/*!
	@brief  Sets the Output Data Rate
*/
/**************************************************************************/
void Adafruit_L3GD20_Unified::setOutputDataRate(gyroDataRate odr)
{
	byte existing;
	read8(GYRO_REGISTER_CTRL_REG1, &existing);
	write(GYRO_REGISTER_CTRL_REG1, (existing &= ~(3 << 6)));
	write(GYRO_REGISTER_CTRL_REG1, (existing |= odr << 6));
}

/**************************************************************************/
/*!
	@brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_L3GD20_Unified::getGyro(sensors_vec_t* gyro)
{
	bool readingValid = false;

	/* Clear the event */
	memset(gyro, 0, sizeof(sensors_vec_t));

	while (!readingValid)
	{
		/* Read 6 bytes from the sensor */
		const uint8_t bytesToRead = 6;
		if (_wire->read(L3GD20_ADDRESS, GYRO_REGISTER_OUT_X_L | 0x80, bytesToRead))
		{
			// Error.
			return false;
		}

		uint8_t xlo = _wire->receive();
		uint8_t xhi = _wire->receive();
		uint8_t ylo = _wire->receive();
		uint8_t yhi = _wire->receive();
		uint8_t zlo = _wire->receive();
		uint8_t zhi = _wire->receive();

		/* Shift values to create properly formed integer (low byte first) */
		gyro->x = (int16_t)(xlo | (xhi << 8));
		gyro->y = (int16_t)(ylo | (yhi << 8));
		gyro->z = (int16_t)(zlo | (zhi << 8));

		/* Make sure the sensor isn't saturating if auto-ranging is enabled */
		readingValid = true;
		if (_autoRangeEnabled)
		{
			/* Check if the sensor is saturating or not */
			if ((gyro->x >= 32760) | (gyro->x <= -32760) |
				(gyro->y >= 32760) | (gyro->y <= -32760) |
				(gyro->z >= 32760) | (gyro->z <= -32760))
			{
				/* Saturating .... increase the range if we can */
				switch (_range)
				{
					case GYRO_RANGE_500DPS:
						/* Push the range up to 2000dps */
						_range = GYRO_RANGE_2000DPS;
						write(GYRO_REGISTER_CTRL_REG1, 0x00);
						write(GYRO_REGISTER_CTRL_REG1, 0x0F);
						write(GYRO_REGISTER_CTRL_REG4, 0x20);
						write(GYRO_REGISTER_CTRL_REG5, 0x80);
						readingValid = false;
						break;
					case GYRO_RANGE_250DPS:
						/* Push the range up to 500dps */
						_range = GYRO_RANGE_500DPS;
						write(GYRO_REGISTER_CTRL_REG1, 0x00);
						write(GYRO_REGISTER_CTRL_REG1, 0x0F);
						write(GYRO_REGISTER_CTRL_REG4, 0x10);
						write(GYRO_REGISTER_CTRL_REG5, 0x80);
						readingValid = false;
						break;
					default:
						break;
				}
			}
		}
	}

	/* Compensate values depending on the resolution */
	switch (_range)
	{
	case GYRO_RANGE_250DPS:
		gyro->x *= GYRO_SENSITIVITY_250DPS;
		gyro->y *= GYRO_SENSITIVITY_250DPS;
		gyro->z *= GYRO_SENSITIVITY_250DPS;
		break;
	case GYRO_RANGE_500DPS:
		gyro->x *= GYRO_SENSITIVITY_500DPS;
		gyro->y *= GYRO_SENSITIVITY_500DPS;
		gyro->z *= GYRO_SENSITIVITY_500DPS;
		break;
	case GYRO_RANGE_2000DPS:
		gyro->x *= GYRO_SENSITIVITY_2000DPS;
		gyro->y *= GYRO_SENSITIVITY_2000DPS;
		gyro->z *= GYRO_SENSITIVITY_2000DPS;
		break;
	}

	/* Convert values to rad/s */
	gyro->x *= SENSORS_DPS_TO_RADS;
	gyro->y *= SENSORS_DPS_TO_RADS;
	gyro->z *= SENSORS_DPS_TO_RADS;

	return true;
}
