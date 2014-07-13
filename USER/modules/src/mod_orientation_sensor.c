/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_orientation_sensor.c
** Descriptions:            GY85 sensor service for orientation measurements
**
**--------------------------------------------------------------------------------------------------------
** Created by:              
** Created date:            
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#include <stdio.h>
#include <stdbool.h>

#include "mod_orientation_sensor.h"
#include "mod_i2c.h"


/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define ADXL345_ADDRESS         0x53 // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
REGISTERS
-----------------------------------------------------------------------*/
#define ADXL345_REG_DEVID               (0x00)    // Device ID
#define ADXL345_REG_THRESH_TAP          (0x1D)
#define ADXL345_REG_OFSX                (0x1E)
#define ADXL345_REG_OFSY                (0x1F)
#define ADXL345_REG_OFSZ                (0x20)
#define ADXL345_REG_DUR                 (0x21)
#define ADXL345_REG_LATENT              (0x22)
#define ADXL345_REG_WINDOW              (0x23)
#define ADXL345_REG_TAP_AXES            (0x2A)
#define ADXL345_REG_BW_RATE             (0x2C)    // Data rate
#define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
#define ADXL345_REG_INT_ENABLE          (0x2E)
#define ADXL345_REG_INT_MAP             (0x2F)
#define ADXL345_REG_INT_SOURCE          (0x30)
#define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
#define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
#define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
#define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
#define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
#define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
#define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
                                        
#define ADXL345_INT_TAP                  0x40 // Enable interrupt when device is tapped once
#define ADXL345_INT_DATA                 0x80 // Enable interrupt when data is available
#define ADXL345_X_TAP_AXIS               0x04
#define ADXL345_Y_TAP_AXIS               0x02
#define ADXL345_Z_TAP_AXIS               0x01
#define ADXL345_DTAP_SUPPRESS_BIT        0x08

#define ADXL345_OFFSET_X                 (-0.07f)
#define ADXL345_OFFSET_Y                 0.07f
#define ADXL345_OFFSET_Z                 0.16f
#define ADXL345_MG2G_MULTIPLIER          (0.004)   // 4mg per lsb
/* Constants */
#define SENSORS_GRAVITY_EARTH           (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON            (1.6F) /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN             (275.0F) /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD        (SENSORS_GRAVITY_EARTH)


#define GYRO_ADDRESS            0x68

#define GYRO_REG_WHOAMI         0x00
#define GYRO_REG_SMPLRT_DIV     0x15
#define GYRO_REG_DLPF_FS        0x16
#define GYRO_REG_INT_CFG        0x17
#define GYRO_REG_INT_STS        0X1A

#define GYRO_REG_TEMP_H         0x1B
#define GYRO_REG_TEMP_L         0x1C
#define GYRO_REG_X_H            0x1D
#define GYRO_REG_X_L            0x1E
#define GYRO_REG_Y_H            0x1F
#define GYRO_REG_Y_L            0x20
#define GYRO_REG_Z_H            0x21
#define GYRO_REG_Z_L            0x22
#define GYRO_REG_PWR_MGM        0x3E

#define GYRO_INT_READY          0x04 // Enable interrupt when device is ready
#define GYRO_INT_DATA           0x01 // Enable interrupt when data is available

#define GYRO_OFFSET_X           19
#define GYRO_OFFSET_Y           (-7)
#define GYRO_OFFSET_Z           (-16)
#define GYRO_SENSITIVITY        14.375F

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} VectorInt_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G = 3, // +/- 16g
  ADXL345_RANGE_8_G  = 2, // +/- 8g
  ADXL345_RANGE_4_G  = 1, // +/- 4g
  ADXL345_RANGE_2_G  = 0  // +/- 2g (default value)
} range_t;

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ = 0x0F, // 1600Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_1600_HZ = 0x0E, // 800Hz Bandwidth 90�A IDD
  ADXL345_DATARATE_800_HZ  = 0x0D, // 400Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_400_HZ  = 0x0C, // 200Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_200_HZ  = 0x0B, // 100Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_100_HZ  = 0x0A, // 50Hz Bandwidth 140�A IDD
  ADXL345_DATARATE_50_HZ   = 0x09, // 25Hz Bandwidth 90�A IDD
  ADXL345_DATARATE_25_HZ   = 0x08, // 12.5Hz Bandwidth 60�A IDD
  ADXL345_DATARATE_12_5_HZ = 0x07, // 6.25Hz Bandwidth 50�A IDD
  ADXL345_DATARATE_6_25HZ  = 0x06, // 3.13Hz Bandwidth 45�A IDD
  ADXL345_DATARATE_3_13_HZ = 0x05, // 1.56Hz Bandwidth 40�A IDD
  ADXL345_DATARATE_1_56_HZ = 0x04, // 0.78Hz Bandwidth 34�A IDD
  ADXL345_DATARATE_0_78_HZ = 0x03, // 0.39Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_39_HZ = 0x02, // 0.20Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_20_HZ = 0x01, // 0.10Hz Bandwidth 23�A IDD
  ADXL345_DATARATE_0_10_HZ = 0x00  // 0.05Hz Bandwidth 23�A IDD (default value)
} dataRate_t;

static uint8_t ITG3205_getDeviceID(void);
static uint8_t ITG3205_powerMgmt(uint8_t sett);
static uint8_t ITG3205_SampleRateDiv(uint8_t sett);
static uint8_t ITG3205_LowPassFilter(uint8_t sett);
static uint8_t ITG3205_InterruptConf(uint8_t sett);
static bool ITG3205_isInterruptRawDataReady(void);
static bool ITG3205_isInterruptDeviceReady(void);
static void ITG3205_getXYZ(VectorInt_t *vi);
static int16_t ITG3205_getX(void);
static int16_t ITG3205_getY(void);
static int16_t ITG3205_getZ(void);

static uint8_t ADXL_getDeviceID(void);
static uint8_t ADXL_powerMgmt(uint8_t sett);
static uint8_t ADXL_getRange(void);
static void ADXL_setRange(uint8_t range);
dataRate_t ADXL_getDataRate(void);
static uint8_t ADXL_setDataRate(dataRate_t dataRate);
static uint8_t ADXL_setInterrupts(uint8_t interrupts);
static uint8_t ADXL_getInterruptStatus(void);
static void ADXL_getXYZ(VectorInt_t *vi);
static int16_t ADXL_getX(void);
static int16_t ADXL_getY(void);
static int16_t ADXL_getZ(void);
static uint8_t ADXL_setTapThreshold(double threshold);
//static double ADXL_getTapThreshold(void);
static uint8_t ADXL_setTapDuration(double duration);
//static double ADXL_getTapDuration(void);
static void ADXL_setAxesTapDetection(bool state, uint8_t axes);
//static uint8_t ADXL_getAxesTapDetection(void);
static void ADXL_setAxesOffset(const Vector_t *v);
static void ADXL_getAxesOffset(Vector_t *v);

static uint32_t constrain(int32_t x, int32_t a, int32_t b);


void vOrientation_sensor_configuration(void)
{
    uint8_t ADXL_deviceID = 0, ITG3205_deviceID = 0;
    
    ADXL_deviceID = ADXL_getDeviceID();
    if(ADXL_deviceID == 0xE5) {
        Vector_t xyzOffsetVect = { ADXL345_OFFSET_X, ADXL345_OFFSET_Y, ADXL345_OFFSET_Z };

        ADXL_powerMgmt(0x08);
        ADXL_setRange(ADXL345_RANGE_4_G);
        ADXL_setDataRate(ADXL345_DATARATE_25_HZ);
        ADXL_setInterrupts(ADXL345_INT_TAP|ADXL345_INT_DATA);
        ADXL_setTapThreshold(4);
        ADXL_setTapDuration(0.05);
        ADXL_setAxesTapDetection(true, ADXL345_Y_TAP_AXIS | ADXL345_Z_TAP_AXIS);
        ADXL_setAxesOffset(&xyzOffsetVect);
    }
    else {
        // sensor not detected 
        // ToDo: implement error service for this case
    }
    
    ITG3205_deviceID = ITG3205_getDeviceID();
    if(ITG3205_deviceID == 0x68) {
        ITG3205_powerMgmt(0);
        ITG3205_SampleRateDiv(0x31);
        ITG3205_LowPassFilter(0x1E);
        ITG3205_InterruptConf(0x05);
    }
    else {
        // sensor is not detected
        // ToDo: implement error service for the case
    }
}

void gyro_get_data(Vector_t *data)
{
    while(ITG3205_isInterruptRawDataReady() != true);

    if(data != NULL) {
        VectorInt_t v = {0};
    
        ITG3205_getXYZ(&v);
        data->x = (v.x + GYRO_OFFSET_X)/GYRO_SENSITIVITY;
        data->y = (v.y + GYRO_OFFSET_Y)/GYRO_SENSITIVITY;
        data->z = (v.z + GYRO_OFFSET_Z)/GYRO_SENSITIVITY;
    }
}

void accl_get_data(AcclData_t *data)
{
    uint8_t status = 0;
    uint8_t event = 0;
    
    while(1) {
        status = ADXL_getInterruptStatus();
        if( (status & ADXL345_INT_TAP) == ADXL345_INT_TAP ) {
           event = 1;
        }
        if( (status & ADXL345_INT_DATA) == ADXL345_INT_DATA ) {
            break;
        }
    }

    if(data != NULL) {
        VectorInt_t v = {0};

        ADXL_getXYZ(&v);
        data->vect.x = v.x * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        data->vect.y = v.y * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        data->vect.z = v.z * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        data->event = event; 
    }
}


static uint8_t ITG3205_getDeviceID(void)
{
    // Check device ID register
     uint8_t reg[2];
     reg[0] = GYRO_REG_WHOAMI;
     xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 1);
     return reg[1];
}

static uint8_t ITG3205_powerMgmt(uint8_t sett)
{
    uint8_t buff[2];
    buff[0] =  GYRO_REG_PWR_MGM;
    buff[1] =  sett;
    return xI2C_write_sequence(GYRO_ADDRESS, buff, 2);
}

static uint8_t ITG3205_SampleRateDiv(uint8_t sett)
{
    uint8_t buff[2];
    buff[0] =  GYRO_REG_SMPLRT_DIV;
    buff[1] =  sett;
    return xI2C_write_sequence(GYRO_ADDRESS, buff, 2);
}

static uint8_t ITG3205_LowPassFilter(uint8_t sett)
{
    uint8_t buff[2];
    buff[0] =  GYRO_REG_DLPF_FS;
    buff[1] =  sett;
    return xI2C_write_sequence(GYRO_ADDRESS, buff, 2);
}

static uint8_t ITG3205_InterruptConf(uint8_t sett)
{
    uint8_t buff[2];
    buff[0] =  GYRO_REG_INT_CFG;
    buff[1] =  sett & 0xF5;
    return xI2C_write_sequence(GYRO_ADDRESS, buff, 2);
}

static bool ITG3205_isInterruptRawDataReady(void)
{
    uint8_t reg[2];
    reg[0] = GYRO_REG_INT_STS;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 1);
    return (reg[1] & GYRO_INT_DATA) == GYRO_INT_DATA;
}

static bool ITG3205_isInterruptDeviceReady(void)
{
    uint8_t reg[2];
    reg[0] = GYRO_REG_INT_STS;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 1);
    return (reg[1] & GYRO_INT_READY) == GYRO_INT_READY;
}

static void ITG3205_getXYZ(VectorInt_t *vi)
{
    uint8_t reg[7];
    reg[0] = GYRO_REG_X_H;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 6);

    vi->x = reg[2] | (reg[1]<<8);
    vi->y = reg[4] | (reg[3]<<8);
    vi->z = reg[6] | (reg[5]<<8);
}

static int16_t ITG3205_getX(void)
{
    uint8_t reg[3];
    reg[0] = GYRO_REG_X_H;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 2);
    return reg[2] | (reg[1]<<8);
}

static int16_t ITG3205_getY(void)
{
    uint8_t reg[3];
    reg[0] = GYRO_REG_Y_H;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 2);
    return reg[2] | (reg[1]<<8);
}

static int16_t ITG3205_getZ(void)
{
    uint8_t reg[3];
    reg[0] = GYRO_REG_Z_H;
    xI2C_read_sequence(GYRO_ADDRESS, reg, 1, &reg[1], 2);
    return reg[2] | (reg[1]<<8);
}

static uint8_t ADXL_getDeviceID(void)
{
    // Check device ID register
    uint8_t reg[2];
    reg[0] = ADXL345_REG_DEVID;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return reg[1];
}

static uint8_t ADXL_powerMgmt(uint8_t sett)
{
    uint8_t buff[2];
    buff[0] =  ADXL345_REG_POWER_CTL;
    buff[1] =  sett;
    return xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
}

static uint8_t ADXL_getRange(void)
{
    uint8_t reg[2];
    reg[0] = ADXL345_REG_DATA_FORMAT;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return (reg[1]&0x03);
}

static void ADXL_setRange(uint8_t range)
{
    uint8_t reg[2];
    uint8_t format;
    reg[0] = ADXL345_REG_DATA_FORMAT;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);

    format = reg[1] & (~0x0F);
    format |= range;
    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    reg[0] = ADXL345_REG_DATA_FORMAT;
    reg[1] = format;
    xI2C_write_sequence(ADXL345_ADDRESS, reg, 2);
}

dataRate_t ADXL_getDataRate(void)
{
    uint8_t reg[2];
    reg[0] = ADXL345_REG_BW_RATE;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return (dataRate_t)(reg[1]&0x0F);
}

static uint8_t ADXL_setDataRate(dataRate_t dataRate)
{
    /* Note: The LOW_POWER bits are currently ignored and we always keep
       the device in 'normal' mode */
    uint8_t buff[2];
    buff[0] = ADXL345_REG_BW_RATE;
    buff[1] = dataRate;
    return xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
}

static uint8_t ADXL_setInterrupts(uint8_t interrupts)
{
    uint8_t buff[2];
    
    buff[0] = ADXL345_REG_INT_MAP;
    buff[1] = 0x00;
    xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
    
    buff[0] = ADXL345_REG_INT_ENABLE;
    buff[1] = interrupts;
    return xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
}

static uint8_t ADXL_getInterruptStatus(void)
{
    uint8_t reg[2];
    reg[0] = ADXL345_REG_INT_SOURCE;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return reg[1];
}

static void ADXL_getXYZ(VectorInt_t *vi)
{
    uint8_t reg[7];
    reg[0] = ADXL345_REG_DATAX0;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 6);

    vi->x = reg[1] | (reg[2]<<8);
    vi->y = reg[3] | (reg[4]<<8);
    vi->z = reg[5] | (reg[6]<<8);
}

static int16_t ADXL_getX(void)
{
    uint8_t reg[3];
    reg[0] = ADXL345_REG_DATAX0;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 2);
    return reg[1] | (reg[2]<<8);
}

static int16_t ADXL_getY(void)
{
    uint8_t reg[3];
    reg[0] = ADXL345_REG_DATAY0;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 2);
    return reg[1] | (reg[2]<<8);
}

static int16_t ADXL_getZ(void)
{
    uint8_t reg[3];
    reg[0] = ADXL345_REG_DATAZ0;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 2);
    return reg[1] | (reg[2]<<8);
}

static uint8_t ADXL_setTapThreshold(double threshold)
{
    uint8_t buff[2];
    uint8_t value = 0;
    
    value = constrain(threshold / 0.0625f, 0, 255);
    buff[0] = ADXL345_REG_THRESH_TAP;
    buff[1] = value;
    return xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
}

// Get Tap Threshold (62.5mg / LSB)
double ADXL_getTapThreshold(void)
{
    uint8_t reg[2];
    reg[0] = ADXL345_REG_THRESH_TAP;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return reg[1] * 0.0625f;
}

// Set Tap Duration (625us / LSB)
static uint8_t ADXL_setTapDuration(double duration)
{
    uint8_t buff[2];
    uint8_t value = 0;
    
    value = constrain(duration / 0.000625f, 0, 255);
    buff[0] = ADXL345_REG_DUR;
    buff[1] = value;
    return xI2C_write_sequence(ADXL345_ADDRESS, buff, 2);
}

// Get Tap Duration (625us / LSB)
double ADXL_getTapDuration(void)
{
    uint8_t reg[2];
    reg[0] = ADXL345_REG_DUR;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);
    return reg[1] * 0.000625f;
}

static void ADXL_setAxesTapDetection(bool state, uint8_t axes)
{
    uint8_t reg[2];
    uint8_t value = 0;

    if (state)
    {
        if(axes > 7) {
            // enable all axes
            value = (ADXL345_X_TAP_AXIS|ADXL345_Y_TAP_AXIS|ADXL345_Z_TAP_AXIS);
        }
        else {
            value = axes;
        }
        
        value |= ADXL345_DTAP_SUPPRESS_BIT;      // suppress double tap, do not need this event
    }
    else {
        value = reg[1] & 0xF8;
    }

    reg[0] = ADXL345_REG_TAP_AXES;
    reg[1] = value;
    xI2C_write_sequence(ADXL345_ADDRESS, reg, 2);
}

uint8_t ADXL_getAxesTapDetection(void)
{
    uint8_t reg[2];
    uint8_t value;
    reg[0] = ADXL345_REG_TAP_AXES;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 1);

    return reg[1];
}

static void ADXL_setAxesOffset(const Vector_t *v)
{
    uint8_t buff[4];
    int32_t value = 0;

    buff[0] = ADXL345_REG_OFSX;

    value = (int32_t)(v->x/0.015625f);
    value = constrain( (uint8_t)value, 0, 255 );
    buff[1] = value;

    value = (int32_t)(v->y/0.015625f);
    value = constrain( (uint8_t)value, 0, 255 );
    buff[2] = value;
    
    value = (int32_t)(v->z/0.015625f);
    value = constrain( (uint8_t)value, 0, 255 );
    buff[3] = value;

    xI2C_write_sequence(ADXL345_ADDRESS, buff, 4);
}

void ADXL_getAxesOffset(Vector_t *v)
{
    uint8_t reg[4];
    reg[0] = ADXL345_REG_OFSX;
    xI2C_read_sequence(ADXL345_ADDRESS, reg, 1, &reg[1], 3);
    
    v->x = (int8_t)reg[1] * 0.015625f;
    v->y = (int8_t)reg[2] * 0.015625f;
    v->z = (int8_t)reg[3] * 0.015625f;
}


//auxiliary functions
static uint32_t constrain(int32_t x, int32_t a, int32_t b)
{
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
