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
const struct device *imu = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(accelerometer));

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

/// @brief Convert the sensor struct to a single physical vaulue
/// @param inSensorValue input struct to be converted
/// @param outValue pointer to output variable
/// @return Error code
static ErrCode_t convertToPhysicalValue( struct sensor_value inSensorValue, float * const outValue )
{
    ErrCode_t errCode = ErrCode_Internal;

    if( !outValue )
    {
        LOG_ERR( "Failed to convert sensor value to physical value, invalid pointer!" );
        goto exit;
    }

    *outValue = ( float )inSensorValue.val1 + ( float )inSensorValue.val2/SENSOR_VALUE2_SCALE;

    errCode = ErrCode_Success;
exit:
    return errCode;
}

/// @brief Interrupt triggered when accelerometer data is ready to be proessed
/// @param dev Pointer to the device
/// @param trig Sensor trigger data
static void processAccelData(const struct device *dev, const struct sensor_trigger *trig)
{
    struct sensor_value buffer;
    float x, y, z;

    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &buffer);
    convertToPhysicalValue( buffer, &x );

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &buffer);
    convertToPhysicalValue( buffer, &y );

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &buffer);
    convertToPhysicalValue( buffer, &z );

    LOG_DBG( "X: %f Y: %f Z: %f, in m/s2", x, y, z );
}

/// @brief Initialize ImuManager instance
/// @return Error code
ErrCode_t ImuManager::init( void )
{
    ErrCode_t errCode = ErrCode_NotInitialized;
    int zephyrCode = -ENOTSUP;

    if( !imu )
    {
        LOG_ERR( "IMU disabled!" );
        goto exit;
    }

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

    LOG_INF( "IMU initialized!" );

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
    
    zephyrCode = sensor_attr_set( imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &samplingFrequencyConfig );

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