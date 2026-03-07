/*
* File: flashbuffer.h
* Author: Daniel Bishara
* Date: March 7, 2026
* Description: declare class methods for the FlashBuffer class
*
* Requires in prj.conf:
*   CONFIG_FCB=y
*   CONFIG_FLASH=y
*   CONFIG_FLASH_MAP=y
*   CONFIG_FLASH_PAGE_LAYOUT=y
*
* Requires a 'flash_buffer_partition' partition defined in the board overlay, e.g.:
*   &flash0 {
*       partitions {
*           flash_buffer_partition: partition@<offset> {
*               label = "flash_buffer_partition";
*               reg = <0x<offset> 0x4000>; // 16KB minimum (4 x 4KB sectors)
*           };
*       };
*   };
*/

#pragma once

#include <zephyr/fs/fcb.h>
#include <zephyr/storage/flash_map.h>

#include "errorCode.h"

#define FLASH_BUFFER_SECTOR_COUNT ( 4 )

class FlashBuffer
{
public:
    static FlashBuffer& Instance( void ) { static FlashBuffer instance; return instance; }
    ErrCode_t init( void );
    ErrCode_t push( const void *inData, uint16_t len );
    ErrCode_t pop( void *outData, uint16_t *outLen );
    bool      isEmpty( void );
    void      clear( void );

private:
    FlashBuffer( void ) = default;
    ~FlashBuffer( void ) = default;
    bool isInit = false;
    struct fcb fcb;
    struct flash_sector sectors[FLASH_BUFFER_SECTOR_COUNT];
    const struct flash_area *fap = nullptr;
    struct fcb_entry readPos;
    bool hasUnread = false;
};
