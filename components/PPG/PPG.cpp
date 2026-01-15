/*
* File: PPG.cpp
* Author: Daniel Bishara
* Date: January 14, 2025
* Description: define class methods for the PPG class
* Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30101.pdf
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "PPG.h"

LOG_MODULE_REGISTER( PpgManager, CONFIG_LOG_DEFAULT_LEVEL );

const struct device *ppg = DEVICE_DT_GET(DT_NODELABEL(ppg));

ErrCode_t PpgManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;

    if( !device_is_ready( ppg ) ) 
    { 
        LOG_ERR( "PPG not ready!" );
        errCode = ErrCode_NotReady; 
        goto exit; 
    }

    errCode = ErrCode_Success;

exit:
    return errCode;
}