/*
* File: PPG.h
* Author: Daniel Bishara
* Date: January 14, 2025
* Description: declare class methods for the PPG class
*/

#pragma once

#include "errorCode.h"

class PpgManager
{
private:
    PpgManager( void ) = default;
    ~PpgManager( void ) = default;
public:
    static PpgManager& Instance( void ) { static PpgManager instance; return instance; }
    ErrCode_t init( void );
};