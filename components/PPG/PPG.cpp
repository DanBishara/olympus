/*
* File: PPG.cpp
* Author: Daniel Bishara
* Date: January 14, 2025
* Description: define class methods for the PPG class
* Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30101.pdf
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "PPG.h"

LOG_MODULE_REGISTER( PpgManager, CONFIG_LOG_DEFAULT_LEVEL );

const struct device *ppg = DEVICE_DT_GET(DT_NODELABEL(ppg));
const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c30));

// PPG is configured with an ADC resolution of 18 bits, so the max value is 2^18 - 1
ErrCode_t PpgManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;

    if( !device_is_ready( ppg ) ) 
    { 
        LOG_ERR( "PPG not ready!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }

    if( !device_is_ready( i2c ) ) 
    { 
        LOG_ERR( "I2C not ready!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }

    errCode = ErrCode_Success;

exit:
    return errCode;
}

ErrCode_t PpgManager::getSensorData( int * outData )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value ppgData;

    if( !outData )
    {
        LOG_ERR( "Invalid pointer for output data!" );
        goto exit;
    }

    zephyrCode = sensor_sample_fetch_chan( ppg, SENSOR_CHAN_RED );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to fetch PPG sample!" );
        goto exit;
    }

    zephyrCode = sensor_channel_get( ppg, SENSOR_CHAN_RED, &ppgData );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to get PPG data!" );
        goto exit;
    }

    *outData = ppgData.val1;

    errCode = ErrCode_Success;
exit:
    return errCode;
}