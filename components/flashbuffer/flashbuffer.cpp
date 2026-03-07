/*
* File: flashbuffer.cpp
* Author: Daniel Bishara
* Date: March 7, 2026
* Description: define class methods for the FlashBuffer class
*
* Uses Zephyr's Flash Circular Buffer (FCB) to persist entries in internal flash.
* Entries are stored sequentially; the FCB automatically overwrites the oldest
* sector when the flash is full, maintaining circular-buffer semantics.
*
* Pop semantics: a RAM-resident read cursor (readPos) tracks the next unread entry.
* Already-read entries remain in flash until the FCB auto-rotates their sector on
* a future push. This is normal behaviour for flash circular buffers.
*/

#include <zephyr/fs/fcb.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include "flashbuffer.h"

LOG_MODULE_REGISTER( FlashBuffer, CONFIG_LOG_DEFAULT_LEVEL );

#define FLASH_BUFFER_PARTITION_ID DT_FIXED_PARTITION_ID( DT_NODELABEL( storage_partition ) )
#define FLASH_BUFFER_MAGIC        ( 0xFB00FB00 )
#define FLASH_BUFFER_VERSION      ( 1 )

ErrCode_t FlashBuffer::init( void )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode;
    uint32_t sectorCount = FLASH_BUFFER_SECTOR_COUNT;

    if ( isInit )
    {
        LOG_WRN( "FlashBuffer already initialized!" );
        errCode = ErrCode_Success;
        goto exit;
    }

    zephyrCode = flash_area_open( FLASH_BUFFER_PARTITION_ID, &fap );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to open flash area! (%d)", zephyrCode );
        goto exit;
    }

    zephyrCode = flash_area_get_sectors( FLASH_BUFFER_PARTITION_ID, &sectorCount, sectors );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to get flash sectors! (%d)", zephyrCode );
        goto exit;
    }

    fcb.f_magic      = FLASH_BUFFER_MAGIC;
    fcb.f_version    = FLASH_BUFFER_VERSION;
    fcb.f_sector_cnt = ( uint8_t )sectorCount;
    fcb.f_sectors    = sectors;

    zephyrCode = fcb_init( FLASH_BUFFER_PARTITION_ID, &fcb );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to initialize FCB! (%d)", zephyrCode );
        goto exit;
    }

    // Set read cursor to the oldest existing entry (if any)
    memset( &readPos, 0, sizeof( readPos ) );
    hasUnread = ( fcb_getnext( &fcb, &readPos ) == 0 );

    LOG_INF( "FlashBuffer initialized with %u sectors", sectorCount );

    isInit = true;
    errCode = ErrCode_Success;
exit:
    return errCode;
}

// @brief Append one entry to the flash circular buffer
// @param inData pointer to the data to write
// @param len    number of bytes to write (max 255)
// @return Error code
ErrCode_t FlashBuffer::push( const void *inData, uint16_t len )
{
    ErrCode_t errCode = ErrCode_Internal;
    struct fcb_entry entry;
    int zephyrCode;

    if ( !isInit ) { LOG_ERR( "FlashBuffer not initialized!" ); goto exit; }
    if ( !inData  ) { LOG_ERR( "Invalid data pointer!" );       goto exit; }

    zephyrCode = fcb_append( &fcb, len, &entry );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to start FCB append! (%d)", zephyrCode );
        goto exit;
    }

    zephyrCode = flash_area_write( fap, FCB_ENTRY_FA_DATA_OFF( entry ), inData, len );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to write to flash! (%d)", zephyrCode );
        goto exit;
    }

    zephyrCode = fcb_append_finish( &fcb, &entry );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to finish FCB append! (%d)", zephyrCode );
        goto exit;
    }

    // If the buffer was empty before this push, point the read cursor at this entry
    if ( !hasUnread )
    {
        memset( &readPos, 0, sizeof( readPos ) );
        hasUnread = ( fcb_getnext( &fcb, &readPos ) == 0 );
    }

    LOG_DBG( "FlashBuffer: pushed %u bytes", len );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

// @brief Read and consume the oldest unread entry
// @param outData pointer to the destination buffer
// @param outLen  set to the number of bytes read
// @return Error code
ErrCode_t FlashBuffer::pop( void *outData, uint16_t *outLen )
{
    ErrCode_t errCode = ErrCode_Internal;
    int zephyrCode;
    struct fcb_entry next;

    if ( !isInit  ) { LOG_ERR( "FlashBuffer not initialized!" ); goto exit; }
    if ( !outData ) { LOG_ERR( "Invalid output pointer!" );       goto exit; }
    if ( !outLen  ) { LOG_ERR( "Invalid length pointer!" );       goto exit; }

    if ( !hasUnread )
    {
        LOG_WRN( "FlashBuffer is empty!" );
        goto exit;
    }

    zephyrCode = flash_area_read( fap, FCB_ENTRY_FA_DATA_OFF( readPos ), outData, readPos.fe_data_len );
    if ( zephyrCode != 0 )
    {
        LOG_ERR( "Failed to read from flash! (%d)", zephyrCode );
        goto exit;
    }

    *outLen = readPos.fe_data_len;

    // Advance the read cursor to the next entry
    next = readPos;
    if ( fcb_getnext( &fcb, &next ) == 0 )
    {
        readPos = next;
    }
    else
    {
        hasUnread = false;
    }

    LOG_DBG( "FlashBuffer: popped %u bytes", *outLen );

    errCode = ErrCode_Success;
exit:
    return errCode;
}

bool FlashBuffer::isEmpty( void )
{
    if ( !isInit ) { return true; }
    return !hasUnread;
}

// @brief Erase all entries from the flash circular buffer
void FlashBuffer::clear( void )
{
    if ( !isInit ) { LOG_ERR( "FlashBuffer not initialized!" ); return; }

    fcb_clear( &fcb );
    memset( &readPos, 0, sizeof( readPos ) );
    hasUnread = false;

    LOG_INF( "FlashBuffer cleared!" );
}
