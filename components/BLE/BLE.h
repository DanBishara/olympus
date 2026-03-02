/*
* File: BLE.h
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: declare class methods for the BLE class
*/

#pragma once 

#include "errorCode.h"

class BLEManager
{
private:
    BLEManager( void ) = default;
    ~BLEManager( void ) = default;
public:
    ErrCode_t init( void );
    static BLEManager& Instance( void ) { static BLEManager instance; return instance; }
};  