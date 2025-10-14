/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "accelerometer.h"

LOG_MODULE_REGISTER( ImuManager, CONFIG_LOG_DEFAULT_LEVEL );


static const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

ErrCode_t ImuManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;

    if( !device_is_ready( imu ) ) 
    { 
        LOG_ERR( "Failed to init IMU!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }

    errCode = ErrCode_Success;

exit:
    return errCode;
}