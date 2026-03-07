/*
* File: main.cpp
* Author: Daniel Bishara
* Date: October 14, 2025
* Description: main entry point for the application. App implementation should be done in the Implementation class
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "accelerometer.h"
#include "LED.h"
#include "PPG.h"
#include "BLE.h"
#include "boost.h"

LOG_MODULE_REGISTER( main, CONFIG_LOG_DEFAULT_LEVEL );

int main(void)
{
    int data=0;
    float ppgCurrent = 0.0f;
    float smoothedCurrent = 0.0f;
    float baselineCurrent = 0.0f;
    float baselineCorrectedCurrent = 0.0f;

    BoostManager::Instance().init();
    ImuManager::Instance().init();
    PpgManager::Instance().init();
    BLEManager::Instance().init();
    DebugLEDManager::Instance().init();
    while(1)
    {
        // PpgManager::Instance().getSensorData( &data );
        // ppgCurrent = ( ( float )data / ( ( 1 << ADC_RESOLUTION_BITS ) - 1 ) ) * MAX30101_FS_RANGE;
        // smoothedCurrent = PpgManager::Instance().rollingAverage( ppgCurrent );
        // baselineCurrent = PpgManager::Instance().calculateBaselineCurrent( ppgCurrent );
        // baselineCorrectedCurrent = ppgCurrent - baselineCurrent;
        // LOG_INF( "PPG Sensor Data: %f", data );
        // LOG_INF( "Smoothed PPG Data: %f", smoothedCurrent );
        // LOG_INF( "Baseline PPG Data: %f", baselineCurrent );
        // LOG_INF( "Baseline-Corrected PPG Data: %f", baselineCorrectedCurrent );
        DebugLEDManager::Instance().toggle();
        k_msleep(2000);
    }

    return 0;
}
