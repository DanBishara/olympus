/*
* File: ringbuffer.h
* Author: Daniel Bishara
* Date: March 5, 2026
* Description: declare class methods for the RingBuffer class
*/

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <stdint.h>

class RingBuffer
{
public:
    RingBuffer( uint8_t *inBuffer, uint16_t inCapacityBytes );
    ~RingBuffer( void ) = default;

    RingBuffer( const RingBuffer& )            = delete;
    RingBuffer& operator=( const RingBuffer& ) = delete;

    bool     push( const void *inData, uint8_t inSize );
    bool     pop( void *outData, uint8_t *outSize );
    bool     peek( void *outData, uint8_t *outSize );
    bool     getNextSize( uint8_t *outSize );
    bool     waitForData( k_timeout_t inTimeout );
    uint16_t size( void );
    uint16_t capacity( void ) const;
    uint16_t spaceAvailable( void );
    bool     isEmpty( void );
    void     clear( void );

private:
    struct ring_buf rb;
    struct k_sem    dataSem;
    struct k_mutex  bufMutex;
    const uint16_t  bufCapacity;
};
