#ifndef BOOT_SHARED_H
#define BOOT_SHARED_H

#include <stdint.h>

#define BOOT_INFO_ADDRESS       0x000000 // Sector 0
#define OTA_AREA_ADDRESS        0x001000 // Sector 1 to 63 (252 KB)
#define BACKUP_AREA_ADDRESS     0x040000 // Sector 64+ (256 KB)

#define BOOT_MAGIC_WORD         0x5A5A5A5C
#define MAX_BOOT_FAIL_COUNT     3

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t current_app_size;
    uint32_t ota_size;
    uint32_t backup_app_size;
    uint8_t boot_failure_count;
    uint8_t is_ota_ready;
    uint8_t iap_request;
    uint8_t reserved[1];
} BootInfo_t;
#pragma pack(pop)

#endif // BOOT_SHARED_H
