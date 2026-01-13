/*
* File: LED.h
* Author: Daniel Bishara
* Date: January 13, 2026
* Description: declare class methods for the debug LED class
*/

#pragma once

#include <zephyr/device.h>

#include "errorCode.h"

class DebugLEDManager
{
private:
    DebugLEDManager( void ) = default;
    ~DebugLEDManager( void ) = default;
    bool isInitalized = false;
public:
    static DebugLEDManager& Instance( void ) { static DebugLEDManager instance; return instance; }
    ErrCode_t init( void );
    ErrCode_t disable( void );
    ErrCode_t enable( void );
    ErrCode_t toggle( void );
};
