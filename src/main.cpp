/*
* File: main.cpp
* Author: Daniel Bishara
* Date: October 14, 2025
* Description: main entry point for the application. App implementation should be done in the Implementation class
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "accelerometer.h"
#include "LED.h"
#include "PPG.h"
#include "BLE.h"
#include "boost.h"
#include "heartrate.h"
#include "stepCounter.h"

LOG_MODULE_REGISTER( main, CONFIG_LOG_DEFAULT_LEVEL );

static const struct gpio_dt_spec irq_pin =  GPIO_DT_SPEC_GET(DT_ALIAS(my_int), gpios);

int main(void)
{
    int state = 0;
    StepCounter::Instance().init();
    BoostManager::Instance().init();
    ImuManager::Instance().init();
    PpgManager::Instance().init();
    BLEManager::Instance().init();
    DebugLEDManager::Instance().init();
    HeartRateManager::Instance().init( MAX30101_EFFECTIVE_SR );
    while(1)
    {
        // PpgManager::Instance().getSensorData( &state );
        // LOG_INF( "PPG Data: %d", state );
        // state = gpio_pin_get_dt( &irq_pin );
        // LOG_INF( "State: %d", state );
        DebugLEDManager::Instance().toggle();
        k_msleep(2000);
    }

    return 0;
}
