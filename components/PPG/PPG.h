/*
* File: PPG.h
* Author: Daniel Bishara
* Date: January 14, 2025
* Description: declare class methods for the PPG class
*/

#pragma once

#include "errorCode.h"

// From zephyr
#define MAX30101_REG_INT_STS1		0x00
#define MAX30101_REG_INT_STS2		0x01
#define MAX30101_REG_INT_EN1		0x02
#define MAX30101_REG_INT_EN2		0x03
#define MAX30101_REG_FIFO_WR		0x04
#define MAX30101_REG_FIFO_OVF		0x05
#define MAX30101_REG_FIFO_RD		0x06
#define MAX30101_REG_FIFO_DATA		0x07
#define MAX30101_REG_FIFO_CFG		0x08
#define MAX30101_REG_MODE_CFG		0x09
#define MAX30101_REG_SPO2_CFG		0x0a
#define MAX30101_REG_LED1_PA		0x0c
#define MAX30101_REG_LED2_PA		0x0d
#define MAX30101_REG_LED3_PA		0x0e
#define MAX30101_REG_PILOT_PA		0x10
#define MAX30101_REG_MULTI_LED		0x11
#define MAX30101_REG_TINT		    0x1f
#define MAX30101_REG_TFRAC		    0x20
#define MAX30101_REG_TEMP_CFG		0x21
#define MAX30101_REG_PROX_INT		0x30
#define MAX30101_REG_REV_ID		    0xfe
#define MAX30101_REG_PART_ID		0xff

#define MAX30101_INT_EN_BIT_A_FULL	( 1 << 7 )

// ADC range in nano amps
#if CONFIG_MAX30101_ADC_RGE == 0
    #define MAX30101_FS_RANGE ( 2048.0f )
#elif CONFIG_MAX30101_ADC_RGE == 1
    #define MAX30101_FS_RANGE ( 4096.0f )
#elif CONFIG_MAX30101_ADC_RGE == 2
    #define MAX30101_FS_RANGE ( 8192.0f )
#elif CONFIG_MAX30101_ADC_RGE == 3
    #define MAX30101_FS_RANGE ( 16384.0f )
#endif

#define ADC_RESOLUTION_BITS ( 18 )


class PpgManager
{
private:
    PpgManager( void ) = default;
    ~PpgManager( void ) = default;
    float dataBuffer[5]; // Will store the most recent 5 samples, used for calculating rolling average to smooth out the data
    float baselineCurrentBuffer[100]; // Buffer to store the most recent 100 samples for calculating the baseline current, which can be used to detect if the sensor is not in contact with the skin
public:
    static PpgManager& Instance( void ) { static PpgManager instance; return instance; }
    ErrCode_t init( void );
    ErrCode_t getSensorData( int * outData );
    float rollingAverage( float inNewSample );
    float calculateBaselineCurrent( float inNewSample );
    void clearInterruptStatus( void );
};