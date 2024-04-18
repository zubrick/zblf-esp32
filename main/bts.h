
// bts.h generated from main/bts.gatt for BTstack
// it needs to be regenerated when the .gatt file is updated. 

// To generate bts.h:
// ../btstack/tool/compile_gatt.py main/bts.gatt bts.h

// att db format version 1

// binary attribute representation:
// - size in bytes (16), flags(16), handle (16), uuid (16/128), value(...)

#include <stdint.h>

// Reference: https://en.cppreference.com/w/cpp/feature_test
#if __cplusplus >= 200704L
constexpr
#endif
const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18, 
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a, 
    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ -'zBLF'
    // READ_ANYBODY
    0x0c, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x7a, 0x42, 0x4c, 0x46, 
    // 0x0004 CHARACTERISTIC-GAP_APPEARANCE - READ | WRITE | DYNAMIC
    0x0d, 0x00, 0x02, 0x00, 0x04, 0x00, 0x03, 0x28, 0x0a, 0x05, 0x00, 0x01, 0x2a, 
    // 0x0005 VALUE CHARACTERISTIC-GAP_APPEARANCE - READ | WRITE | DYNAMIC
    // READ_ANYBODY, WRITE_ANYBODY
    0x08, 0x00, 0x0a, 0x01, 0x05, 0x00, 0x01, 0x2a, 
    // GAP Peripheral Privacy Flag
    // 0x0006 CHARACTERISTIC-2A02 - READ | WRITE | DYNAMIC
    0x0d, 0x00, 0x02, 0x00, 0x06, 0x00, 0x03, 0x28, 0x0a, 0x07, 0x00, 0x02, 0x2a, 
    // 0x0007 VALUE CHARACTERISTIC-2A02 - READ | WRITE | DYNAMIC
    // READ_ANYBODY, WRITE_ANYBODY
    0x09, 0x00, 0x0a, 0x01, 0x07, 0x00, 0x02, 0x2a, 0x00, 
    // 0x0008 PRIMARY_SERVICE-GATT_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x08, 0x00, 0x00, 0x28, 0x01, 0x18, 
    // 0x0009 CHARACTERISTIC-GATT_DATABASE_HASH - READ
    0x0d, 0x00, 0x02, 0x00, 0x09, 0x00, 0x03, 0x28, 0x02, 0x0a, 0x00, 0x2a, 0x2b, 
    // 0x000a VALUE CHARACTERISTIC-GATT_DATABASE_HASH - READ -''
    // READ_ANYBODY
    0x18, 0x00, 0x02, 0x00, 0x0a, 0x00, 0x2a, 0x2b, 0xce, 0xc7, 0xd1, 0xac, 0xa9, 0x9a, 0xfa, 0x35, 0xac, 0xed, 0x3b, 0x30, 0x6e, 0x38, 0xdc, 0x19, 
    // END
    0x00, 0x00, 
}; // total size 74 bytes 


//
// list service handle ranges
//
#define ATT_SERVICE_GAP_SERVICE_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_END_HANDLE 0x0007
#define ATT_SERVICE_GAP_SERVICE_01_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_01_END_HANDLE 0x0007
#define ATT_SERVICE_GATT_SERVICE_START_HANDLE 0x0008
#define ATT_SERVICE_GATT_SERVICE_END_HANDLE 0x000a
#define ATT_SERVICE_GATT_SERVICE_01_START_HANDLE 0x0008
#define ATT_SERVICE_GATT_SERVICE_01_END_HANDLE 0x000a

//
// list mapping between characteristics and handles
//
#define ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_GAP_APPEARANCE_01_VALUE_HANDLE 0x0005
#define ATT_CHARACTERISTIC_2A02_01_VALUE_HANDLE 0x0007
#define ATT_CHARACTERISTIC_GATT_DATABASE_HASH_01_VALUE_HANDLE 0x000a
