#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. Include AP_Common/AP_FWVersion.h instead.
#endif

#include "ap_version.h"

#define THISFIRMWARE "MiniDP V0.1.0-dev"

#define FIRMWARE_VERSION 0,1,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 0
#define FW_MINOR 1
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
#include <AP_Common/AP_FWVersionDefine.h>
