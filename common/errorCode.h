/*
* File: errCode.h
* Author: Daniel Bishara
* Date: October 14, 2025
* Description: define error codes for error handling
*/

#pragma once

typedef enum ErrCode_t
{
    ErrCode_Success         = 0,
    ErrCode_Internal        = -1,
    ErrCode_NotReady        = -2,
    ErrCode_NotInitialized  = -3
}ErrCode_t;