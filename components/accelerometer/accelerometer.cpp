/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
* Datasheet: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <lsm6dso_reg.h>

#include "accelerometer.h"

#define MAX_SAMPLING_FREQ_HZ            ( 6664 )
#define DEFAULT_SAMPLING_FREQUENCY_HZ   ( 12.5 ) // View datasheet for valid frequencies
#define SENSOR_VALUE2_SCALE             ( 1000000 ) // val2 in sensor_value is the decimal portion * 10^6
#define G_ACCEL_MS2                     ( 9.81 )
#define DEFAULT_FULL_SCALE_MS2          ( 2 * G_ACCEL_MS2 )
#define TRIGGER_X_MS2                   ( 2 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Y_MS2                   ( 1 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Z_MS2                   ( 1 * G_ACCEL_MS2 ) // in m/(s^2)

LOG_MODULE_REGISTER( ImuManager, CONFIG_LOG_DEFAULT_LEVEL );

// TODO: The zephyr driver doesn't make use of the ML core, will likely need to add drivers for that if found to be useful
const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));
stmdev_ctx_t * config = (stmdev_ctx_t *)imu->config;

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

ErrCode_t ImuManager::setThreshold( uint8_t inAxis, float inThreshold )
{
    ErrCode_t errCode = ErrCode_Internal;
    return errCode;
}

ErrCode_t ImuManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;

    if( !device_is_ready( imu ) ) 
    { 
        LOG_ERR( "IMU not ready!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }
    
    errCode = setSamplingFrequency( DEFAULT_SAMPLING_FREQUENCY_HZ );
    if( errCode ){ goto exit; }

    errCode = setFullScaleRange( DEFAULT_FULL_SCALE_MS2 );
    if( errCode ){ goto exit; }

    errCode = ErrCode_Success;

exit:
    return errCode;
}

// Full scale range is init in the device tree
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

// sampling frequency is set in devicetree
// max frequency is 6664 Hz
ErrCode_t ImuManager::setSamplingFrequency( uint16_t inSamplingFrequency )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value samplingFrequencyConfig;

    if( inSamplingFrequency > MAX_SAMPLING_FREQ_HZ ){ inSamplingFrequency = MAX_SAMPLING_FREQ_HZ; }

    errCode = convertToSensorValue( inSamplingFrequency, &samplingFrequencyConfig );
    if( errCode ) { goto exit; }
    
    zephyrCode = sensor_attr_set( imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &samplingFrequencyConfig );

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