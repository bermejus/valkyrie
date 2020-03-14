#pragma once

#include <cstdint>

#define SST26_16MBIT_SIZE       0x200000
#define SST26_32MBIT_SIZE       0x400000
#define SST26_64MBIT_SIZE       0x800000

struct jedec_id_t
{
    union {
        struct {
            uint8_t manufacturer_id;
            uint8_t device_type;
            uint8_t device_id;
        };
        uint8_t data[3];
    };
};

struct flash_cmd_t
{
    uint8_t cmd;
    uint8_t address_h;
    uint8_t address_m;
    uint8_t address_l;
};

void flash_init();
void flash_read(uint32_t address, uint8_t* data, uint32_t length);
void flash_write(uint32_t address, const uint8_t* data, uint32_t length);