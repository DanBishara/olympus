/*
* File: BOOST.h
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: declare class methods for the BOOST class
*/

#pragma once

#include "errorCode.h"

class boostManager
{
private:
    boostManager( void ) = default;
    ~boostManager( void ) = default;
public:
    ErrCode_t init( void );
    void enableBoost( void );
    void disableBoost( void );
    void select5V( void );
    void select3V( void );
    static boostManager& Instance( void ) { static boostManager instance; return instance; }
}; 