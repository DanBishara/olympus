/*
* File: LED.cpp
* Author: Daniel Bishara
* Date: January 13, 2026
* Description: define class methods for the debug LED class 
*/

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "LED.h"

LOG_MODULE_REGISTER( LED, CONFIG_LOG_DEFAULT_LEVEL );

#define ENABLE_LED      ( 1 )
#define DISABLE_LED     ( 0 )

/* The devicetree node identifier for the "led0" alias. */
#define DEBUG_LED DT_ALIAS( led0 )

#if DT_NODE_HAS_STATUS(DEBUG_LED, okay)
    // This only compiles if the status is "okay"
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DEBUG_LED, gpios);
#else
    #warning "Debug LED node is disabled, skipping GPIO initialization."
    static const struct gpio_dt_spec led = {0}; 
#endif

/// @brief Initalize the debug LED manager
/// @param void
/// @return Error code
ErrCode_t DebugLEDManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    if( isInitalized ) { goto exit; }
    if ( !gpio_is_ready_dt( &led ) ) { goto exit; }

    ret = gpio_pin_configure_dt( &led, GPIO_OUTPUT_ACTIVE );
	if ( ret ) { goto exit; }

    LOG_INF( "Debug LED init Success" );

    isInitalized = true;

    errCode = ErrCode_Success;

exit:
    return errCode;
}

/// @brief Disable the debugging LED
/// @param void
/// @return Error code
ErrCode_t DebugLEDManager::disable( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    if( !isInitalized ) { errCode = ErrCode_NotReady; goto exit; }

    ret = gpio_pin_set_dt(&led, DISABLE_LED);
    if ( ret ) { goto exit; }

    LOG_INF( "Disabling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}

/// @brief Enable the debugging LED
/// @param void
/// @return Error code
ErrCode_t DebugLEDManager::enable( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    if( !isInitalized ) { errCode = ErrCode_NotReady; goto exit; }

    ret = gpio_pin_set_dt( &led, ENABLE_LED );
    if ( ret ) { goto exit; }

    LOG_INF( "Enabling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}

/// @brief Toggle the debugging LED
/// @param void
/// @return Error code
ErrCode_t DebugLEDManager::toggle( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int ret;

    if( !isInitalized ) { errCode = ErrCode_NotReady; goto exit; }

    ret = gpio_pin_toggle_dt( &led );
    if ( ret ) { goto exit; }

    LOG_DBG( "Toggling debug LED!" );

    errCode = ErrCode_Success;
    
exit:
    return errCode;
}
