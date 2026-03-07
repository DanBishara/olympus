/*
* File: boost.cpp
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: define class methods for the boost class
*/
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "boost.h"

LOG_MODULE_REGISTER( BoostManager, CONFIG_LOG_DEFAULT_LEVEL );

#if DT_NODE_HAS_STATUS(DT_ALIAS(boost_en), okay)
static const struct gpio_dt_spec boost_en_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(boost_en), gpios);
#else
static const struct gpio_dt_spec boost_en_pin = {0};
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(boost_sel), okay)
static const struct gpio_dt_spec boost_sel_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(boost_sel), gpios);
#else
static const struct gpio_dt_spec boost_sel_pin = {0};
#endif

ErrCode_t BoostManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;

    if ( isInit )
    {
        LOG_WRN( "BoostManager already initialized!" );
        errCode = ErrCode_Success;
        goto exit;
    }

    if ( !gpio_is_ready_dt( &boost_en_pin ) )
    {
        LOG_ERR( "GPIO device %s is not ready!", boost_en_pin.port->name );
        errCode = ErrCode_NotReady;
        goto exit;
    }

    if ( !gpio_is_ready_dt( &boost_sel_pin ) )
    {
        LOG_ERR( "GPIO device %s is not ready!", boost_sel_pin.port->name );
        errCode = ErrCode_NotReady;
        goto exit;
    }

    zephyrCode = gpio_pin_configure_dt( &boost_en_pin, GPIO_OUTPUT_ACTIVE );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to configure GPIO pin!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    zephyrCode = gpio_pin_configure_dt( &boost_sel_pin, GPIO_OUTPUT_INACTIVE );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to configure GPIO pin!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    LOG_INF( "Boost converter initialized!" );

    isInit = true;
    errCode = ErrCode_Success;
exit:
    return errCode;
}

void BoostManager::enableBoost( void )
{
    if ( !isInit ) { LOG_ERR( "BoostManager not initialized!" ); return; }
    gpio_pin_set_dt( &boost_en_pin, 1 );
}

void BoostManager::disableBoost( void )
{
    if ( !isInit ) { LOG_ERR( "BoostManager not initialized!" ); return; }
    gpio_pin_set_dt( &boost_en_pin, 0 );
}

void BoostManager::select5V( void )
{
    if ( !isInit ) { LOG_ERR( "BoostManager not initialized!" ); return; }
    gpio_pin_set_dt( &boost_sel_pin, 0 );
}

void BoostManager::select3V( void )
{
    if ( !isInit ) { LOG_ERR( "BoostManager not initialized!" ); return; }
    gpio_pin_set_dt( &boost_sel_pin, 1 );
}
