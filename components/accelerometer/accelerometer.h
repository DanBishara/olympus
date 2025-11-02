/*
* File: accelerometer.h
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: declare class methods for the imu class
*/

#pragma once

#include <zephyr/device.h>
#include <lsm6dso_reg.h>

#include "errorCode.h"

class ImuManager
{
private:
    ImuManager( void ) = default;
    ~ImuManager( void ) = default;
    ErrCode_t setWakeupThreshold( uint8_t inThresholdPercent );
    ErrCode_t enableInterrupt( void );
public:
    static ImuManager& Instance( void ) { static ImuManager instance; return instance; }
    ErrCode_t init( void );
    ErrCode_t setFullScaleRange ( float inFullScaleRange );
    ErrCode_t setSamplingFrequency( uint16_t inSamplingFrequency );
    ErrCode_t setPowerMode( lsm6dso_xl_hm_mode_t inPowerMode );
};