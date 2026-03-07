/*
* File: boost.h
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: declare class methods for the BoostManager class
*/

#pragma once

#include "errorCode.h"

class BoostManager
{
public:
    ErrCode_t init( void );
    void enableBoost( void );
    void disableBoost( void );
    void select5V( void );
    void select3V( void );
    static BoostManager& Instance( void ) { static BoostManager instance; return instance; }

private:
    BoostManager( void ) = default;
    ~BoostManager( void ) = default;
};
