/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: define class methods for the imu class
* Datasheet: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
*/

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "LED.h"

LOG_MODULE_REGISTER( LED, LOG_LEVEL_INF );

#define ENABLE_LED      ( 1 )
#define DISABLE_LED     ( 0 )

/* The devicetree node identifier for the "led0" alias. */
#define DEBUG_LED DT_ALIAS( led0 )

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DEBUG_LED, gpios);

ErrCode_t DebugLEDManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    if ( !gpio_is_ready_dt( &led ) ) { goto exit; }

    ret = gpio_pin_configure_dt( &led, GPIO_OUTPUT_ACTIVE );
	if ( ret ) { goto exit; }

    LOG_INF( "Debug LED init Success" );

    errCode = ErrCode_Success;

exit:
    return errCode;
}

ErrCode_t DebugLEDManager::disable( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    ret = gpio_pin_set_dt(&led, DISABLE_LED);
    if ( ret ) { goto exit; }

    LOG_INF( "Disabling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}


ErrCode_t DebugLEDManager::enable( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    ret = gpio_pin_set_dt( &led, ENABLE_LED );
    if ( ret ) { goto exit; }

    LOG_INF( "Enabling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}

ErrCode_t DebugLEDManager::toggle( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    ret = gpio_pin_toggle_dt( &led );
    if ( ret ) { goto exit; }

    LOG_INF( "Toggling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}