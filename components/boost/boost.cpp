/*
* File: boost.cpp
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: define class methods for the boost class
*/
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "boost.h"

LOG_MODULE_REGISTER( boostManager, CONFIG_LOG_DEFAULT_LEVEL );

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

ErrCode_t boostManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;

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

    zephyrCode = gpio_pin_configure_dt( &boost_sel_pin, GPIO_OUTPUT_ACTIVE );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to configure GPIO pin!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    select5V(); // default to 5V output
    enableBoost(); // enable boost by default

    LOG_INF( "Boost converter initialized!" );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

void boostManager::enableBoost( void )
{
    gpio_pin_set_dt( &boost_en_pin, 1 );
}

void boostManager::disableBoost( void )
{
    gpio_pin_set_dt( &boost_en_pin, 0 );
}

void boostManager::select5V( void )
{
    gpio_pin_set_dt( &boost_sel_pin, 0 );
} 

void boostManager::select3V( void )
{
    gpio_pin_set_dt( &boost_sel_pin, 1 );
}