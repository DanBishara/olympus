/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "accelerometer.h"

#define SAMPLING_FREQUENCY_HZ   ( 10 )

LOG_MODULE_REGISTER( ImuManager, CONFIG_LOG_DEFAULT_LEVEL );

// TODO: The zephyr driver doesn't make use of the ML core, will likely need to add drivers for that if found to be useful
static const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

ErrCode_t ImuManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value samplingConfig = { .val1 = SAMPLING_FREQUENCY_HZ, .val2 = 0 };

    if( !device_is_ready( imu ) ) 
    { 
        LOG_ERR( "IMU not ready!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }
    
    zephyrCode = sensor_attr_set( imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &samplingConfig );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to set IMU sampling frequency!" );
        errCode = ErrCode_Internal;
        goto exit;
    }


    errCode = ErrCode_Success;

exit:
    return errCode;
}