/*
* File: ringbuffer.cpp
* Author: Daniel Bishara
* Date: March 5, 2026
* Description: define class methods for the RingBuffer class
*
* Each message stored in the buffer is prefixed with a 1-byte header that
* holds the payload length. This means:
*   - Maximum payload size per message is 255 bytes
*   - Each stored message consumes (1 + payloadSize) bytes of capacity
*   - Callers do not need to track message sizes externally
*/

#include <string.h>

#include <zephyr/logging/log.h>

#include "ringbuffer.h"

LOG_MODULE_REGISTER( RingBuffer, CONFIG_LOG_DEFAULT_LEVEL );

// @brief Initialize the ring buffer using a caller-provided static buffer.
//        The caller owns the backing memory and is responsible for ensuring
//        it remains valid for the lifetime of the RingBuffer instance.
// @param inBuffer        Pointer to the static byte array to use as storage
// @param inCapacityBytes Size of inBuffer in bytes
RingBuffer::RingBuffer( uint8_t *inBuffer, uint16_t inCapacityBytes ) : bufCapacity( inCapacityBytes )
{
    ring_buf_init( &rb, inCapacityBytes, inBuffer );
}

// @brief Write a message into the buffer, prefixed with a 1-byte size header.
//        Each message consumes (1 + inSize) bytes of capacity.
// @param inData  Pointer to the payload to write
// @param inSize  Number of payload bytes (max 255)
// @return true on success, false if there is insufficient space
bool RingBuffer::push( const void *inData, uint8_t inSize )
{
    uint16_t required = sizeof(uint8_t) + inSize;

    if( ring_buf_space_get( &rb ) < required )
    {
        LOG_WRN( "push: insufficient space for %u byte message", inSize );
        return false;
    }

    ring_buf_put( &rb, &inSize, sizeof(uint8_t) );
    ring_buf_put( &rb, static_cast<const uint8_t *>( inData ), inSize );

    return true;
}

// @brief Read and consume the next message from the buffer.
//        The payload size is read from the 1-byte header; the caller does
//        not need to know the size in advance. outData must be large enough
//        to hold the payload — call getNextSize() first if unsure.
// @param outData   Destination for the payload bytes
// @param outSize   Set to the number of bytes written into outData
// @return true on success, false if the buffer is empty
bool RingBuffer::pop( void *outData, uint8_t *outSize )
{
    uint8_t header;

    if( ring_buf_get( &rb, &header, sizeof(uint8_t) ) != sizeof(uint8_t) )
    {
        return false;
    }

    if( ring_buf_get( &rb, static_cast<uint8_t *>( outData ), header ) != header )
    {
        LOG_ERR( "pop: payload read failed after header consumed — buffer may be corrupt!" );
        return false;
    }

    *outSize = header;
    return true;
}

// @brief Peek at the next message without consuming it.
//        Reads the header and payload but leaves both in the buffer.
// @param outData   Destination for the payload bytes
// @param outSize   Set to the number of bytes written into outData
// @return true on success, false if the buffer is empty or the message wraps
//         across the end of the internal buffer (call pop instead in that case)
bool RingBuffer::peek( void *outData, uint8_t *outSize )
{
    uint8_t  header;
    uint8_t *ptr;

    // Claim a contiguous view of header + payload without consuming
    uint32_t claimed = ring_buf_get_claim( &rb, &ptr, sizeof(uint8_t) );
    if( claimed < sizeof(uint8_t) )
    {
        ring_buf_get_finish( &rb, 0 );
        return false;
    }

    header = ptr[0];

    ring_buf_get_finish( &rb, 0 );

    // Claim again for the full message to get the payload pointer
    claimed = ring_buf_get_claim( &rb, &ptr, sizeof(uint8_t) + header );
    if( claimed < (uint32_t)( sizeof(uint8_t) + header ) )
    {
        ring_buf_get_finish( &rb, 0 );
        LOG_WRN( "peek: message wraps internal buffer boundary, use pop instead" );
        return false;
    }

    memcpy( outData, ptr + sizeof(uint8_t), header );
    ring_buf_get_finish( &rb, 0 );

    *outSize = header;
    return true;
}

// @brief Peek at the size of the next message without consuming any data.
// @param outSize  Set to the payload size of the next message in bytes
// @return true on success, false if the buffer is empty
bool RingBuffer::getNextSize( uint8_t *outSize )
{
    return ring_buf_peek( &rb, outSize, sizeof(uint8_t) ) == sizeof(uint8_t);
}

// @brief Total bytes currently stored in the buffer (including headers)
uint16_t RingBuffer::size( void )
{
    return static_cast<uint16_t>( ring_buf_size_get( &rb ) );
}

// @brief Total byte capacity the buffer was constructed with
uint16_t RingBuffer::capacity( void ) const
{
    return bufCapacity;
}

// @brief Number of free bytes remaining
uint16_t RingBuffer::spaceAvailable( void )
{
    return static_cast<uint16_t>( ring_buf_space_get( &rb ) );
}

// @brief True if the buffer contains no messages
bool RingBuffer::isEmpty( void )
{
    return ring_buf_is_empty( &rb );
}

// @brief Discard all data in the buffer
void RingBuffer::clear( void )
{
    ring_buf_reset( &rb );
}
