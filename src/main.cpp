/*
* File: main.cpp
* Author: Daniel Bishara
* Date: October 14, 2025
* Description: main entry point for the application. App implementation should be done in the Implementation class
*/

#include <zephyr/kernel.h>
#include "accelerometer.h"
#include "LED.h"

int main(void)
{
    ImuManager::Instance().init();
    DebugLEDManager::Instance().init();
    k_msleep(2000);
    DebugLEDManager::Instance().disable();
    k_msleep(2000);
    DebugLEDManager::Instance().enable();
    k_msleep(2000);
    DebugLEDManager::Instance().toggle();
    return 0;
}
