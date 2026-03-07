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

// @brief work handler for processing PPG data when an interrupt is triggered, will read the data from the sensor, calculate a rolling average and baseline current, and log the results
// @param work pointer to the work item, not used in this handler but required by the k_work API
void ppgWorkHandler( struct k_work *work )
{
    int data = 0;
    float ppgCurrent = 0; 
    float smoothedCurrent = 0;
    float baselineCurrent = 0;
    float baselineCorrectedCurrent = 0;
    int64_t timestamp = k_uptime_get();
    PpgManager::Instance().getSensorData( &data );

    ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE; // in nano amps
    smoothedCurrent = PpgManager::Instance().rollingAverage( ppgCurrent );
    baselineCurrent = PpgManager::Instance().calculateBaselineCurrent( ppgCurrent );
    baselineCorrectedCurrent = ppgCurrent - baselineCurrent;
    LOG_INF( "PPG Data: %d, Current: %f nA, Smoothed: %f nA, Baseline: %f nA, Baseline Corrected: %f nA", data, ppgCurrent, smoothedCurrent, baselineCurrent, baselineCorrectedCurrent );

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

    memset( dataBuffer, 0, sizeof(dataBuffer) );
    memset( baselineCurrentBuffer, 0, sizeof(baselineCurrentBuffer) );

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

// @brief Read data from the MAX30101 sensor, this should only be called from the work handler when an interrupt is triggered, as it will read the data from the sensor which can take some time
// @param outData pointer to an integer where the raw sensor data will be stored
// @return Error code
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

    LOG_DBG( "PPG Data: %d", *outData );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

// @brief Calculate a rolling average of the PPG current, this is used to smooth out the data and reduce noise, it will store the most recent 5 samples and calculate the average of those samples
// @param inNewSample the new sample to be added to the buffer and used in the average calculation
// @return the rolling average of the samples in the buffer
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

// @brief Calculate the baseline current of the PPG signal, this is used to detect if the sensor is not in contact with the skin, as the current will drop significantly when the sensor is not in contact, it will store the most recent 100 samples and calculate the average of those samples
// @param inNewSample the new sample to be added to the buffer and used in the average calculation
// @return the baseline current calculated from the samples in the buffer
float PpgManager::calculateBaselineCurrent( float inNewSample )
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

// @brief Clear the interrupt status of the MAX30101, this is necessary to allow new interrupts to be triggered, as the interrupt will only trigger once until the status is cleared
void PpgManager::clearInterruptStatus( void )
{
    uint8_t buffer = {0};
    i2c_burst_read( i2c, DT_REG_ADDR(DT_NODELABEL(ppg)), MAX30101_REG_INT_STS1, &buffer, sizeof(buffer) );
}