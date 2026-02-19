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
#include <zephyr/kernel.h>

#include "PPG.h"

LOG_MODULE_REGISTER( PpgManager, CONFIG_LOG_DEFAULT_LEVEL );

void max30101_interrupt_handler(const struct device *port, 
                                struct gpio_callback *cb, 
                                uint32_t pins)
{
    int data = 0;
    // TODO: need to create workitem to read data from the sensor and process it, as this handler should be as fast as possible and not do any heavy processing
    LOG_INF("MAX30101 Interrupt Triggered! %d", data);
}

const struct device *ppg = DEVICE_DT_GET(DT_NODELABEL(ppg));
const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c30));
static const struct gpio_dt_spec irq_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(my_int), gpios);

/* This structure holds the callback information for the kernel */
static struct gpio_callback irq_cb_data;

// PPG is configured with an ADC resolution of 18 bits, so the max value is 2^18 - 1
ErrCode_t PpgManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;

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

    if ( !gpio_is_ready_dt( &irq_pin ) )
    {
        LOG_ERR( "GPIO device %s is not ready!", irq_pin.port->name );
        errCode = ErrCode_NotReady;
        goto exit;
    }

    /* Configure the physical pin as input */
    zephyrCode = gpio_pin_configure_dt( &irq_pin, GPIO_INPUT );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to configure GPIO pin!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    /* Configure the interrupt trigger to falling edge */
    zephyrCode = gpio_pin_interrupt_configure_dt( &irq_pin, GPIO_INT_EDGE_FALLING );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to configure GPIO interrupt!" );
        errCode = ErrCode_Internal;
        goto exit;
    }

    /* Initialize the callback structure and add it to the port */
    gpio_init_callback( &irq_cb_data, max30101_interrupt_handler, BIT( irq_pin.pin ) );
    gpio_add_callback( irq_pin.port, &irq_cb_data );

    LOG_INF( "MAX30101 Interrupt Initialized on P1.12" );

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