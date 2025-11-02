/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
* Datasheet: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "accelerometer.h"

#define MAX_SAMPLING_FREQ_HZ            ( 6664 )
#define DEFAULT_SAMPLING_FREQUENCY_HZ   ( 12.5 ) // View datasheet for valid frequencies
#define SENSOR_VALUE2_SCALE             ( 1000000 ) // val2 in sensor_value is the decimal portion * 10^6
#define G_ACCEL_MS2                     ( 9.81 )
#define DEFAULT_FULL_SCALE_MS2          ( 2 * G_ACCEL_MS2 )
#define TRIGGER_X_MS2                   ( 2 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Y_MS2                   ( 1 * G_ACCEL_MS2 ) // in m/(s^2)
#define TRIGGER_Z_MS2                   ( 1 * G_ACCEL_MS2 ) // in m/(s^2)
#define WAKEUP_THRESHOLD_MASK           ( 0x3F )

LOG_MODULE_REGISTER( ImuManager, CONFIG_LOG_DEFAULT_LEVEL );

// TODO: The zephyr driver doesn't make use of the ML core, will likely need to add drivers for that if found to be useful
const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(accelerometer));
stmdev_ctx_t * stDevConfig = (stmdev_ctx_t *)imu->config;

/// @brief Interrupt triggered when accelerometer data is ready to be proessed
/// @param dev Pointer to the device
/// @param trig Sensor trigger data
static void processAccelData(const struct device *dev, const struct sensor_trigger *trig)
{
	
}

/// @brief Convert desired value to the sensor format require by zephyr
/// @param inValue Value to be converter to sensor value
/// @param outSensorValue Pointer to sensor vaue struct
/// @return Error Code
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

    errCode = ErrCode_Success;
exit:
    return errCode;
}

// TODO: this implementation isn't very good
/// @brief Set wake-up threhsold for accelerometer, dependent on WAKE_UP_DUR register and relative to FS range
/// @param inThresholdPercent wake-up threshold in m/s2
/// @return Error code
ErrCode_t ImuManager::setWakeupThreshold( uint8_t inThresholdPercent )
{
    ErrCode_t errCode = ErrCode_Internal;
    int32_t stErrCode = -1;
    float threshold = inThresholdPercent/100;
    lsm6dso_wake_ths_w_t wakeupThresholdWeight;
    uint16_t wakeupResolution;
    uint8_t newThreshold;

    stErrCode = lsm6dso_wkup_ths_weight_get( stDevConfig, &wakeupThresholdWeight );
    if( stErrCode ) { goto exit; }

    wakeupResolution = wakeupThresholdWeight == LSM6DSO_LSb_FS_DIV_64? 64 : 256;

    // Wakeup threshold limited to 6 bits
    newThreshold = ( uint8_t )( wakeupResolution * threshold ) & WAKEUP_THRESHOLD_MASK;

    stErrCode = lsm6dso_wkup_threshold_set( stDevConfig, newThreshold );
    if( stErrCode )
    {
        LOG_ERR( "Failed to set new wake-up threshold" );
        goto exit;
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}

/// @brief Initialize ImuManager instance
/// @return Error code
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

    errCode = setFullScaleRange( DEFAULT_FULL_SCALE_MS2 );
    if( errCode ){ goto exit; }

    errCode = enableInterrupt();
    if( errCode ){ goto exit; }

    errCode = ErrCode_Success;
exit:
    return errCode;
}

/// @brief set the acceleromter full scale analog range in m/s2
/// @param inFullScaleRange Full scale range in m/s2
/// @return Error code
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

/// @brief Set the sampling frequency of the accelerometer, can be set in the device tree
/// @param inSamplingFrequency sampling frequency in Hz
/// @return Error code
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

/// @brief Trigger an interrupt when data is ready to be processed
/// @param void
/// @return Error Code
ErrCode_t ImuManager::enableInterrupt( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_trigger trig;

    errCode = setSamplingFrequency( DEFAULT_SAMPLING_FREQUENCY_HZ );
    if( errCode ) { goto exit; }

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	zephyrCode = sensor_trigger_set( imu, &trig, processAccelData );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to enable IMU interrupt!" );
        goto exit;
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}