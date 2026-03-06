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
*
* Thread safety:
*   bufMutex protects all ring_buf operations from concurrent access.
*   dataSem is signaled after releasing bufMutex so the consumer thread
*   can immediately acquire the mutex upon waking.
*/

#include <string.h>

#include <zephyr/logging/log.h>

#include "ringbuffer.h"

LOG_MODULE_REGISTER( RingBuffer, CONFIG_LOG_DEFAULT_LEVEL );

// @brief Initialize the ring buffer using a caller-provided static buffer.
//        The caller owns the backing memory and is responsible for ensuring
//        it remains valid for the lifetime of the RingBuffer instance.
//        The binary semaphore is initialized unsignaled; it is given once
//        the buffer reaches 50% capacity so a waiting thread can be unblocked.
// @param inBuffer        Pointer to the static byte array to use as storage
// @param inCapacityBytes Size of inBuffer in bytes
RingBuffer::RingBuffer( uint8_t *inBuffer, uint16_t inCapacityBytes ) : bufCapacity( inCapacityBytes )
{
    ring_buf_init( &rb, inCapacityBytes, inBuffer );
    k_sem_init( &dataSem, 0, 1 );
    k_mutex_init( &bufMutex );
}

// @brief Write a message into the buffer, prefixed with a 1-byte size header.
//        Each message consumes (1 + inSize) bytes of capacity.
//        Signals dataSem after releasing bufMutex when the buffer reaches 50% capacity.
// @param inData  Pointer to the payload to write
// @param inSize  Number of payload bytes (max 255)
// @return true on success, false if there is insufficient space
bool RingBuffer::push( const void *inData, uint8_t inSize )
{
    uint16_t required = sizeof(uint8_t) + inSize;
    bool     result   = false;
    bool     doSignal = false;

    k_mutex_lock( &bufMutex, K_FOREVER );

    if( ring_buf_space_get( &rb ) < required )
    {
        LOG_WRN( "push: insufficient space for %u byte message", inSize );
        goto exit;
    }

    ring_buf_put( &rb, &inSize, sizeof(uint8_t) );
    ring_buf_put( &rb, static_cast<const uint8_t *>( inData ), inSize );

    doSignal = ring_buf_size_get( &rb ) >= bufCapacity / 2;
    result   = true;

exit:
    k_mutex_unlock( &bufMutex );

    if( doSignal )
    {
        k_sem_give( &dataSem );
    }

    return result;
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
    uint8_t  header = 0;
    uint32_t read   = 0;
    bool     result = false;

    k_mutex_lock( &bufMutex, K_FOREVER );

    read = ring_buf_get( &rb, &header, sizeof(uint8_t) );
    if( read != sizeof(uint8_t) )
    {
        goto exit;
    }

    read = ring_buf_get( &rb, static_cast<uint8_t *>( outData ), header );
    if( read != header )
    {
        LOG_ERR( "pop: payload read failed after header consumed — buffer may be corrupt!" );
        goto exit;
    }

    *outSize = header;
    result   = true;

exit:
    k_mutex_unlock( &bufMutex );
    return result;
}

// @brief Peek at the next message without consuming it.
//        Reads the header and payload but leaves both in the buffer.
//        Uses ring_buf_peek which handles wrap-around internally.
// @param outData   Destination for the payload bytes
// @param outSize   Set to the number of bytes written into outData
// @return true on success, false if the buffer is empty
bool RingBuffer::peek( void *outData, uint8_t *outSize )
{
    uint8_t  header                              = 0;
    uint32_t read                                = 0;
    // Use static to avoid allocating 256 bytes on the thread stack.
    // Protected by bufMutex, so this is thread-safe.
    static uint8_t temp[ sizeof(uint8_t) + UINT8_MAX ];
    bool     result                              = false;

    k_mutex_lock( &bufMutex, K_FOREVER );

    read = ring_buf_peek( &rb, &header, sizeof(uint8_t) );
    if( read != sizeof(uint8_t) )
    {
        goto exit;
    }

    // ring_buf_peek always reads from position 0, so to skip the header byte
    // we peek the full message (header + payload) into a temp buffer and copy
    // just the payload out. ring_buf_peek handles wrap-around internally.
    read = ring_buf_peek( &rb, temp, sizeof(uint8_t) + header );
    if( read != (uint32_t)( sizeof(uint8_t) + header ) )
    {
        LOG_WRN( "peek: not enough data" );
        goto exit;
    }

    memcpy( outData, temp + sizeof(uint8_t), header );
    *outSize = header;
    result   = true;

exit:
    k_mutex_unlock( &bufMutex );
    return result;
}

// @brief Peek at the size of the next message without consuming any data.
// @param outSize  Set to the payload size of the next message in bytes
// @return true on success, false if the buffer is empty
bool RingBuffer::getNextSize( uint8_t *outSize )
{
    bool result = false;

    k_mutex_lock( &bufMutex, K_FOREVER );
    result = ring_buf_peek( &rb, outSize, sizeof(uint8_t) ) == sizeof(uint8_t);
    k_mutex_unlock( &bufMutex );

    return result;
}

// @brief Block the calling thread until the buffer reaches 50% capacity
//        or the timeout expires. Does not hold bufMutex while waiting so
//        the producer thread can continue pushing data.
// @param inTimeout  Zephyr timeout (e.g. K_FOREVER, K_MSEC(100))
// @return true if signaled, false if timed out
bool RingBuffer::waitForData( k_timeout_t inTimeout )
{
    return k_sem_take( &dataSem, inTimeout ) == 0;
}

// @brief Total bytes currently stored in the buffer (including headers)
uint16_t RingBuffer::size( void )
{
    uint16_t result = 0;

    k_mutex_lock( &bufMutex, K_FOREVER );
    result = static_cast<uint16_t>( ring_buf_size_get( &rb ) );
    k_mutex_unlock( &bufMutex );

    return result;
}

// @brief Total byte capacity the buffer was constructed with
uint16_t RingBuffer::capacity( void ) const
{
    return bufCapacity;
}

// @brief Number of free bytes remaining
uint16_t RingBuffer::spaceAvailable( void )
{
    uint16_t result = 0;

    k_mutex_lock( &bufMutex, K_FOREVER );
    result = static_cast<uint16_t>( ring_buf_space_get( &rb ) );
    k_mutex_unlock( &bufMutex );

    return result;
}

// @brief True if the buffer contains no messages
bool RingBuffer::isEmpty( void )
{
    bool result = false;

    k_mutex_lock( &bufMutex, K_FOREVER );
    result = ring_buf_is_empty( &rb );
    k_mutex_unlock( &bufMutex );

    return result;
}

// @brief Discard all data in the buffer
void RingBuffer::clear( void )
{
    k_mutex_lock( &bufMutex, K_FOREVER );
    ring_buf_reset( &rb );
    k_mutex_unlock( &bufMutex );
}
