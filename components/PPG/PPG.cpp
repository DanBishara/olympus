/*
* File: PPG.cpp
* Author: Daniel Bishara
* Date: January 14, 2025
* Description: define class methods for the PPG class
* Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30101.pdf
*/

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "PPG.h"
#include "heartrate.h"

LOG_MODULE_REGISTER( PpgManager, CONFIG_LOG_DEFAULT_LEVEL );

static struct k_work ppgWorkItem;
static struct gpio_callback irq_cb_data;

const struct device *ppg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ppg));
const struct device *i2c = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c30));
#if DT_NODE_HAS_STATUS(DT_ALIAS(my_int), okay)
static const struct gpio_dt_spec irq_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(my_int), gpios);
#else
static const struct gpio_dt_spec irq_pin = {0};
#endif

// @brief work handler for processing PPG data when an interrupt is triggered, will read the raw data from the sensor and push it to the HeartRateManager
// @param work pointer to the work item, not used in this handler but required by the k_work API
void ppgWorkHandler( struct k_work *work )
{
    int data = 0;
    float ppgCurrent = 0;

    PpgManager::Instance().getRedData( &data );
    ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE;
    LOG_DBG( "Red LED: %d, Current: %f nA", data, ppgCurrent );
    HeartRateManager::Instance().pushRedLedSample( ppgCurrent );

    PpgManager::Instance().getIrData( &data );
    ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE;
    LOG_DBG( "IR LED: %d, Current: %f nA", data, ppgCurrent );
    HeartRateManager::Instance().pushIrLedSample( ppgCurrent );

    PpgManager::Instance().getGreenData( &data );
    ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE;
    LOG_DBG( "Green LED: %d, Current: %f nA", data, ppgCurrent );
    HeartRateManager::Instance().pushGreenLedSample( ppgCurrent );

    PpgManager::Instance().clearInterruptStatus();
}

// @brief Submit work item to read and process PPG data, this is the interrupt handler for the MAX30101 interrupt pin, it will be triggered when new data is available to read from the sensor
// @param port pointer to the GPIO device, not used in this handler but required by the GPIO API
// @param cb pointer to the GPIO callback structure, not used in this handler but required by the GPIO API
// @param pins bitmask of the pins that triggered the interrupt, not used in this handler but required by the GPIO API
void max30101_interrupt_handler(const struct device *port, 
                                struct gpio_callback *cb, 
                                uint32_t pins)
{
    // TODO: need to create workitem to read data from the sensor and process it, as this handler should be as fast as possible and not do any heavy processing
    LOG_DBG("MAX30101 Interrupt Triggered!");
    k_work_submit( &ppgWorkItem );
}

// @brief Initialize PpgManager instance, will configure the interrupt pin and enable the interrupt on the MAX30101
// @return Error code
ErrCode_t PpgManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    int data = 0;

    if( !ppg )
    {
        LOG_ERR( "PPG disabled!" );
        goto exit;
    }

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

    // can't poll sensor in ISR, so need to create a work item to read the data and process it
    k_work_init( &ppgWorkItem, ppgWorkHandler );

    /* Configure the physical pin as input */
    zephyrCode = gpio_pin_configure_dt( &irq_pin, ( GPIO_INPUT | GPIO_PULL_UP ) );
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

    // Need to manually enable interrupt
    i2c_reg_write_byte( i2c, DT_REG_ADDR(DT_NODELABEL(ppg)), MAX30101_REG_INT_EN1, MAX30101_INT_EN_BIT_A_FULL | MAX30101_INT_EN_BIT_PPG_RDY );

    clearInterruptStatus();
    LOG_INF("PPG Init successful");

    errCode = ErrCode_Success;

exit:
    return errCode;
}

// @brief Read data from the MAX30101 sensor for a specific LED channel
// @param outData pointer to an integer where the raw sensor data will be stored
// @param channel the LED channel to read from
// @return Error code
ErrCode_t PpgManager::getSensorData( int * outData, max30101_led_channel channel )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    struct sensor_value ppgData;
    enum sensor_channel sensorChan;

    if( !outData )
    {
        LOG_ERR( "Invalid pointer for output data!" );
        goto exit;
    }

    switch( channel )
    {
        case MAX30101_LED_CHANNEL_RED:   sensorChan = SENSOR_CHAN_RED;   break;
        case MAX30101_LED_CHANNEL_IR:    sensorChan = SENSOR_CHAN_IR;    break;
        case MAX30101_LED_CHANNEL_GREEN: sensorChan = SENSOR_CHAN_GREEN; break;
        default:
            LOG_ERR( "Invalid LED channel!" );
            goto exit;
    }

    zephyrCode = sensor_sample_fetch_chan( ppg, sensorChan );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to fetch PPG sample!" );
        goto exit;
    }

    zephyrCode = sensor_channel_get( ppg, sensorChan, &ppgData );
    if( zephyrCode )
    {
        LOG_ERR( "Failed to get PPG data!" );
        goto exit;
    }

    *outData = ppgData.val1;

    LOG_DBG( "PPG Data: %d", *outData );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

ErrCode_t PpgManager::getRedData( int * outData )
{
    return getSensorData( outData, MAX30101_LED_CHANNEL_RED );
}

ErrCode_t PpgManager::getIrData( int * outData )
{
    return getSensorData( outData, MAX30101_LED_CHANNEL_IR );
}

ErrCode_t PpgManager::getGreenData( int * outData )
{
    return getSensorData( outData, MAX30101_LED_CHANNEL_GREEN );
}

// @brief Clear the interrupt status of the MAX30101, this is necessary to allow new interrupts to be triggered, as the interrupt will only trigger once until the status is cleared
void PpgManager::clearInterruptStatus( void )
{
    uint8_t buffer = {0};
    i2c_burst_read( i2c, DT_REG_ADDR(DT_NODELABEL(ppg)), MAX30101_REG_INT_STS1, &buffer, sizeof(buffer) );
}