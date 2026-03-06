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
#include "stepCounter.h"

#define MAX_ACCEL_SAMPLING_FREQ_HZ      ( 6664 )
#define MAX_GYRO_SAMPLING_FREQ_HZ       ( 6664 )
#define DEFAULT_SAMPLING_FREQUENCY_HZ   ( 12.5 ) // View datasheet for valid frequencies
#define SENSOR_VALUE2_SCALE             ( 1000000 ) // val2 in sensor_value is the decimal portion * 10^6
#define G_ACCEL_MS2                     ( 9.81 )
#define DEFAULT_FULL_SCALE_MS2          ( 2 * G_ACCEL_MS2 )
#define DEFAULT_GYRO_FULL_SCALE_DPS     ( 250.0f ) // ±250 dps covers arm swing (~1-3 rad/s = ~57-172 dps)
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

/// @brief Set the full scale range for the given sensor channel
/// @param inChannel Sensor channel (e.g. SENSOR_CHAN_ACCEL_XYZ, SENSOR_CHAN_GYRO_XYZ)
/// @param inFullScaleRange Full scale range in the channel's native units
/// @return Error code
static ErrCode_t setFullScaleRange( enum sensor_channel inChannel, float inFullScaleRange )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value fullScaleConfig;

    errCode = convertToSensorValue( inFullScaleRange, &fullScaleConfig );
    if( errCode ) { goto exit; }

    zephyrCode = sensor_attr_set( imu, inChannel, SENSOR_ATTR_FULL_SCALE, &fullScaleConfig );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to set full scale range for channel %d!", inChannel );
        errCode = ErrCode_Internal;
        goto exit;
    }

    errCode = ErrCode_Success;
exit:
    return errCode;
}

/// @brief Set the sampling frequency for the given sensor channel
/// @param inChannel Sensor channel (e.g. SENSOR_CHAN_ACCEL_XYZ, SENSOR_CHAN_GYRO_XYZ)
/// @param inSamplingFrequency Sampling frequency in Hz
/// @param inMaxFrequency Maximum allowed frequency for this channel in Hz
/// @return Error code
static ErrCode_t setSamplingFrequency( enum sensor_channel inChannel, uint16_t inSamplingFrequency, uint16_t inMaxFrequency )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value samplingFrequencyConfig;

    if( inSamplingFrequency > inMaxFrequency ){ inSamplingFrequency = inMaxFrequency; }

    errCode = convertToSensorValue( inSamplingFrequency, &samplingFrequencyConfig );
    if( errCode ) { goto exit; }

    zephyrCode = sensor_attr_set( imu, inChannel, SENSOR_ATTR_SAMPLING_FREQUENCY, &samplingFrequencyConfig );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to set sampling frequency for channel %d!", inChannel );
        errCode = ErrCode_Internal;
        goto exit;
    }

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
    float ax, ay, az;
    float gx, gy, gz;

    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &buffer);
    convertToPhysicalValue( buffer, &ax );

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &buffer);
    convertToPhysicalValue( buffer, &ay );

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &buffer);
    convertToPhysicalValue( buffer, &az );

    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &buffer);
    convertToPhysicalValue( buffer, &gx );

    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &buffer);
    convertToPhysicalValue( buffer, &gy );

    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &buffer);
    convertToPhysicalValue( buffer, &gz );

    LOG_DBG( "Accel: %f %f %f m/s2 | Gyro: %f %f %f rad/s", ax, ay, az, gx, gy, gz );

    StepCounter::Instance().pushSample( ax, ay, az, gx, gy, gz );
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

    errCode = setAccelFullScaleRange( DEFAULT_FULL_SCALE_MS2 );
    if( errCode ){ goto exit; }

    errCode = setGyroFullScaleRange( DEFAULT_GYRO_FULL_SCALE_DPS );
    if( errCode ){ goto exit; }

    errCode = setGyroSamplingFrequency( DEFAULT_SAMPLING_FREQUENCY_HZ );
    if( errCode ){ goto exit; }

    errCode = enableInterrupt();
    if( errCode ){ goto exit; }

    LOG_INF( "IMU initialized!" );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

/// @brief Set the accelerometer full scale range in m/s²
/// @param inFullScaleRange Full scale range in m/s²
/// @return Error code
ErrCode_t ImuManager::setAccelFullScaleRange( float inFullScaleRange )
{
    return setFullScaleRange( SENSOR_CHAN_ACCEL_XYZ, inFullScaleRange );
}

/// @brief Set the accelerometer sampling frequency in Hz
/// @param inSamplingFrequency Sampling frequency in Hz
/// @return Error code
ErrCode_t ImuManager::setAccelSamplingFrequency( uint16_t inSamplingFrequency )
{
    return setSamplingFrequency( SENSOR_CHAN_ACCEL_XYZ, inSamplingFrequency, MAX_ACCEL_SAMPLING_FREQ_HZ );
}

/// @brief Set the gyroscope full scale range in degrees per second
/// @param inFullScaleRange Full scale range in degrees per second
/// @return Error code
ErrCode_t ImuManager::setGyroFullScaleRange( float inFullScaleRange )
{
    return setFullScaleRange( SENSOR_CHAN_GYRO_XYZ, inFullScaleRange );
}

/// @brief Set the gyroscope sampling frequency in Hz
/// @param inSamplingFrequency Sampling frequency in Hz
/// @return Error code
ErrCode_t ImuManager::setGyroSamplingFrequency( uint16_t inSamplingFrequency )
{
    return setSamplingFrequency( SENSOR_CHAN_GYRO_XYZ, inSamplingFrequency, MAX_GYRO_SAMPLING_FREQ_HZ );
}

/// @brief Trigger an interrupt when data is ready to be processed
/// @param void
/// @return Error Code
ErrCode_t ImuManager::enableInterrupt( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_trigger trig;

    errCode = setAccelSamplingFrequency( DEFAULT_SAMPLING_FREQUENCY_HZ );
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
