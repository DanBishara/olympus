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
#define SENSOR_VALUE2_SCALE     ( 1000000 ) // val2 in sensor_value is the decimal portion * 10^6
#define G_ACCEL_MS2             ( 9.81 )
#define DEFAULT_FULL_SCALE_MS2  ( 2 * G_ACCEL_MS2 )
#define TRIGGER_X_MS2           ( 2 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Y_MS2           ( 1 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Z_MS2           ( 1 * G_ACCEL_MS2 ) // in m/(s^2)

LOG_MODULE_REGISTER( ImuManager, CONFIG_LOG_DEFAULT_LEVEL );

// TODO: The zephyr driver doesn't make use of the ML core, will likely need to add drivers for that if found to be useful
static const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));

static ErrCode_t convertToSensorValue( float inValue, struct sensor_value * const outSensorValue )
{
    ErrCode_t errCode = ErrCode_Internal;

    if( !outSensorValue )
    {
        LOG_ERR( "Failed to convert input to sensor value, invalid pointer!" );
        goto exit;
    }

    outSensorValue->val1 = ( int )inValue;
    outSensorValue->val2 = ( int )( ( inValue - outSensorValue->val1 ) * SENSOR_VALUE2_SCALE );
exit:
    return errCode;
}

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

    errCode = setFullScaleRange( DEFAULT_FULL_SCALE_MS2 );
    if( errCode ){ goto exit; }

    errCode = ErrCode_Success;

exit:
    return errCode;
}

ErrCode_t ImuManager::setFullScaleRange( float inFullScaleRange )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value fullScaleConfig;

    errCode = convertToSensorValue( inFullScaleRange, &fullScaleConfig );
    if( errCode ) { goto exit; }
    
    zephyrCode = sensor_attr_set( imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fullScaleConfig );

    if( zephyrCode )
    {
        LOG_ERR( "Failed to set IMU full scale range!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}