/*
* File: BOOST.h
* Author: Daniel Bishara
* Date: March 1, 2026
* Description: declare class methods for the BOOST class
*/

#pragma once

class boostManager
{
private:
    boostManager( void ) = default;
    ~boostManager( void ) = default;
public:
    void init( void );
    static boostManager& Instance( void ) { static boostManager instance; return instance; }
}; 