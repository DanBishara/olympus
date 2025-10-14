/*
* File: accelerometer.cpp
* Author: Daniel Bishara
* Date: October 13, 2025
* Description: declare class methods for the imu class
*/

#pragma once

#include <zephyr/device.h>

class ImuManager
{
private:
    ImuManager( void ) = default;
    ~ImuManager( void ) = default;
public:
    static ImuManager& Instance( void ) { static ImuManager instance; return instance; }
    int init( void );
};