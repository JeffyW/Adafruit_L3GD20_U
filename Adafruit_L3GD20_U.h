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

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

/* Datasheet can be found at: https://cdn-shop.adafruit.com/datasheets/L3GD20H.pdf */

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define L3GD20_ADDRESS           (0x6B)        // 1101011
    #define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
    #define L3GD20_ID                (0xD4)
    #define L3GD20H_ID               (0xD7)
    // Sesitivity values from the mechanical characteristics in the datasheet.
    #define GYRO_SENSITIVITY_250DPS  (0.00875F)
    #define GYRO_SENSITIVITY_500DPS  (0.0175F)
    #define GYRO_SENSITIVITY_2000DPS (0.070F)
/*=========================================================================*/

/*=========================================================================
Data Available
0: new data is not yet available
1: new data is available
---------------------------------------------------------------------------*/
#define GYRO_STATUS_XDA   0x01;
#define GYRO_STATUS_YDA   0x02;
#define GYRO_STATUS_ZDA   0x04;
#define GYRO_STATUS_ZYXDA 0x08; // Not sure if this means all or any
/*=========================================================================*/

/*=========================================================================
Axis Data Overrun
0: no overrun has occurred;
1: new data has overwritten the previous one before it was read
---------------------------------------------------------------------------*/
#define GYRO_STATUS_XOR   0x10
#define GYRO_STATUS_YOR   0x20
#define GYRO_STATUS_ZOR   0x40
#define GYRO_STATUS_ZYXOR 0x80 // Not sure if this means all or any
/*=========================================================================*/

/*=========================================================================
   CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1
---------------------------------------------------------------------------*/

// DR1-DR0 (bits 7-6) Output Data Rate selection
// Note that these values vary based on the Low_ODR bit (0) in the LOW_ODR register
// These values assume a value of 0 in Low_ODR. See table 21 in the datasheet if set.
#define GYRO_CTRL1_DR_800Hz 0xC0 // 11
#define GYRO_CTRL1_DR_400Hz 0x80 // 10
#define GYRO_CTRL1_DR_200Hz 0x40 // 01
#define GYRO_CTRL1_DR_100Hz 0x00 // 00

#define GYRO_CTRL1_Power    0x08 // Power mode (default 0).  0: Power down; 1: Normal/Sleep (sleep on 1000)
#define GYRO_CTRL1_ZEn      0x04 // Z axis enabled (default 1)
#define GYRO_CTRL1_YEn      0x02 // Y axis enabled (default 1)
#define GYRO_CTRL1_XEn      0x01 // X axis enabled (defautl 1)

// Helpers
#define GYRO_CTRL1_3AxisEnabled 0x0F
#define GYRO_CTRL1_Disabled     0x00
/*=========================================================================*/

/*=========================================================================
   CTRL_REG3 (0x22)
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
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0
---------------------------------------------------------------------------*/
#define GYRO_CTRL3_INT2_DRDY 0x80

/*=========================================================================*/

/*=========================================================================
   CTRL_REG4 (0x23)
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
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0
---------------------------------------------------------------------------*/
#define GYRO_CTRL4_BDU    0x80 // Block data update (default 0).  If set, output registers won't update until read.
#define GYRO_CTRL4_BLE    0x40 // Big/Little Endian (default 0).  If set, Least Significant Byte occurs before MSB.

// FS1-FS0 (bits 5-4) Full scale selection, defaults to 00
#define GYRO_CTRL4_FS_2000DPS 0x20 // 10
#define GYRO_CTRL4_FS_500DPS  0x10 // 01
#define GYRO_CTRL4_FS_250DPS  0x00 // 00

#define GYRO_CTRL4_FS_IMPen   0x08 // Level sensitive latched enabled
// ST2-ST1 (bits 2-1) Self Test Enabled (defaults to 00 Normal)
#define GYRO_CTRL4_FS_ST_Test1  0x06 // Self test 1(-)
#define GYRO_CTRL4_FS_ST_Test0  0x02 // Self test 0(+)
#define GYRO_CTRL4_FS_ST_Normal 0x00 // Normal mode

#define GYRO_CTRL4_FS_SIM       0x01 // SPI Serial Interface Mode selection.  0: 4-wire; 1: 3-wire

/*=========================================================================*/

/*=========================================================================
   CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00
---------------------------------------------------------------------------*/
#define GYRO_CTRL5_BOOT    0x80
#define GYRO_CTRL5_FIFO_EN 0x40
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
      GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
      GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
      GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
      GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
      GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
      GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
      GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
      GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
      GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
      GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
      GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
      GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
      GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
      GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
      GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
      GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
      GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
      GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
      GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
      GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
      GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
      GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
      GYRO_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
    } gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      GYRO_RANGE_250DPS  = GYRO_CTRL4_FS_250DPS,
      GYRO_RANGE_500DPS  = GYRO_CTRL4_FS_500DPS,
      GYRO_RANGE_2000DPS = GYRO_CTRL4_FS_2000DPS
    } gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    OUTPUT DATA RATE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      GYRO_ODR_95                       = GYRO_CTRL1_DR_100Hz, // 95 Hz
      GYRO_ODR_190                      = GYRO_CTRL1_DR_200Hz, // 190 Hz
      GYRO_ODR_380                      = GYRO_CTRL1_DR_400Hz, // 380 Hz
      GYRO_ODR_760                      = GYRO_CTRL1_DR_800Hz // 760 Hz
    } gyroDataRate;
/*=========================================================================*/

// Angles are measured first in Degrees Per Second (DSP) then converted to Radians Per Second.
typedef struct
{
    uint8_t temp;
    uint8_t status;
    float x;
    float y;
    float z;
} gyro_event_t;

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    /** Encapsulates a single raw data sample from the sensor. */
    typedef struct gyroRawData_s
    {
        /** The X axis data. */
        int16_t x;
        /** The Y axis data. */
        int16_t y;
        /** The Z axis data. */
        int16_t z;
    } gyroRawData_t;
/*=========================================================================*/

/**
 * Driver for the Adafruit L3GD20 3-Axis gyroscope.
 */
class Adafruit_L3GD20_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_L3GD20_Unified(int32_t sensorID = -1);

    bool begin           ( gyroRange_t rng = GYRO_RANGE_250DPS, TwoWire *theWire=&Wire);
    void enableAutoRange ( bool enabled );
    void enableDRDYInterrupt ( bool enabled );
    void setOutputDataRate ( gyroDataRate odr );
    bool getEvent        ( sensors_event_t* );
    bool getEvent        ( gyro_event_t* );
    void getSensor       ( sensor_t* );

    /** Raw sensor data from the last successful read event. */
    gyroRawData_t raw;

  private:
    void        write8  ( byte reg, byte value );
    byte        read8   ( byte reg );
    gyroRange_t _range;
    int32_t     _sensorID;
    bool        _autoRangeEnabled;
};

/* Non Unified (old) driver for compatibility reasons */
typedef gyroRange_t     l3gd20Range_t;
typedef gyroRegisters_t l3gd20Registers_t;

/**
 * Encapsulates a single XYZ data sample from the sensor.
 */
typedef struct l3gd20Data_s
{
  /** Data from the X axis. */
  float x;
  /** Data from the Y axis. */
  float y;
  /** Data from the Z axis. */
  float z;
} l3gd20Data;

/// @private
class Adafruit_L3GD20
{
  public:
    Adafruit_L3GD20(int8_t cs, int8_t mosi, int8_t miso, int8_t clk);
    Adafruit_L3GD20(void);

    bool begin ( l3gd20Range_t rng=GYRO_RANGE_250DPS, byte addr=L3GD20_ADDRESS );
    void read  ( void );

    l3gd20Data data;    // Last read will be available here

  private:
    void write8     ( l3gd20Registers_t reg, byte value );
    byte read8      ( l3gd20Registers_t reg );
    uint8_t SPIxfer ( uint8_t x );

    byte          address;
    l3gd20Range_t range;
    int8_t        _miso, _mosi, _clk, _cs;
};
#endif
