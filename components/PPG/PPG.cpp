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
#include <zephyr/drivers/i2c.h>

#include "PPG.h"

LOG_MODULE_REGISTER( PpgManager, LOG_LEVEL_INF );

static struct k_work ppgWorkItem;

const struct device *ppg = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(ppg));
const struct device *i2c = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c30));
#if DT_NODE_HAS_STATUS(DT_ALIAS(my_int), okay)
static const struct gpio_dt_spec irq_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(my_int), gpios);
#else
static const struct gpio_dt_spec irq_pin = {0};
#endif

/* This structure holds the callback information for the kernel */
static struct gpio_callback irq_cb_data;

void ppgWorkHandler( struct k_work *work )
{
    int data = 0;
    uint8_t buffer = {0};
    int64_t timestamp = k_uptime_get();
    PpgManager::Instance().getSensorData( &data );

    float ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE; // in nano amps
    // LOG_INF( "%ld %d %f", timestamp, data, ppgCurrent );
    float smoothedCurrent = PpgManager::Instance().rollingAverage( ppgCurrent );
    LOG_INF( "PPG Data: %d, Current: %f nA", data, ppgCurrent );

    // Need to read the interrupt status register to clear the interrupt, otherwise it'll only trigger once
    i2c_burst_read( i2c, DT_REG_ADDR(DT_NODELABEL(ppg)), MAX30101_REG_INT_STS1, &buffer, sizeof(buffer) );
}

void max30101_interrupt_handler(const struct device *port, 
                                struct gpio_callback *cb, 
                                uint32_t pins)
{
    // TODO: need to create workitem to read data from the sensor and process it, as this handler should be as fast as possible and not do any heavy processing
    LOG_DBG("MAX30101 Interrupt Triggered!");
    k_work_submit( &ppgWorkItem );
}

// PPG is configured with an ADC resolution of 18 bits, so the max value is 2^18 - 1
ErrCode_t PpgManager::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode = -ENOTSUP;
    int data = 0;

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

    memset( dataBuffer, 0, sizeof(dataBuffer) );

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
    i2c_reg_write_byte( i2c, DT_REG_ADDR(DT_NODELABEL(ppg)), MAX30101_REG_INT_EN1, MAX30101_INT_EN_BIT_A_FULL );

    LOG_INF( "MAX30101 Interrupt Initialized on P1.12" );

    getSensorData( &data );
    LOG_INF( "INIT PPG Data: %d", data );

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

    LOG_INF( "PPG Data: %d", *outData );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

float PpgManager::rollingAverage( float inNewSample )
{
    // Shift all the old samples back one position
    for( int i = sizeof(dataBuffer)/sizeof(dataBuffer[0]) - 1; i > 0; i-- )
    {
        dataBuffer[i] = dataBuffer[i-1];
    }
    // Add the new sample to the front of the buffer
    dataBuffer[0] = inNewSample;

    // Calculate the average of the samples in the buffer
    float sum = 0;
    for( int i = 0; i < sizeof(dataBuffer)/sizeof(dataBuffer[0]); i++ )
    {
        sum += dataBuffer[i];
    }
    return sum / ( sizeof(dataBuffer)/sizeof(dataBuffer[0]) );
}

flaot PpgManager::calculateBaselineCurrent( float inNewSample )
{
    // Shift all the old samples back one position
    for( int i = sizeof(baselineCurrentBuffer)/sizeof(baselineCurrentBuffer[0]) - 1; i > 0; i-- )
    {
        baselineCurrentBuffer[i] = baselineCurrentBuffer[i-1];
    }
    // Add the new sample to the front of the buffer
    baselineCurrentBuffer[0] = inNewSample;

    // Calculate the average of the samples in the buffer
    float sum = 0;
    for( int i = 0; i < sizeof(baselineCurrentBuffer)/sizeof(baselineCurrentBuffer[0]); i++ )
    {
        sum += baselineCurrentBuffer[i];
    }
    return sum / ( sizeof(baselineCurrentBuffer)/sizeof(baselineCurrentBuffer[0]) );
}