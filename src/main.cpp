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

LOG_MODULE_REGISTER( main, CONFIG_LOG_DEFAULT_LEVEL );

int main(void)
{
    int data = 0;
    ImuManager::Instance().init();
    PpgManager::Instance().init();
    BLEManager::Instance().init();
    DebugLEDManager::Instance().init();
    while(1)
    {
        DebugLEDManager::Instance().toggle();
        k_msleep(2000);
    }

    return 0;
}
