/*
* File: main.cpp
* Author: Daniel Bishara
* Date: October 14, 2025
* Description: main entry point for the application. App implementation should be done in the Implementation class
*/

#include <zephyr/kernel.h>
#include "accelerometer.h"
#include "LED.h"
#include "PPG.h"

int main(void)
{
    DebugLEDManager::Instance().init();
    while(1)
    {
        DebugLEDManager::Instance().toggle();
        k_msleep(2000);
    }

    return 0;
}
