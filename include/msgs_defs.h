//
// Copyright 2020 i-RoCS
//

#ifndef MSGS_DEFS_H_
#define MSGS_DEFS_H_

#include <stdint.h>
#include <vector>
#include <zlib.h>

namespace Sentinel
{
#pragma pack(push, 1)
struct start_msg
{
    uint32_t crc = 0;
    uint32_t seq_number = 0;
    uint64_t reserved = 0;
    uint32_t op_code = 0x35;
    uint32_t ip;
    uint16_t port;

    uint8_t device_enabled = 0b1000;
    uint8_t intensity_enabled = 0;
    uint8_t point_in_safety_enabled = 0;
    uint8_t active_zone_set_enabled = 0;
    uint8_t io_pin_enabled = 0;
    uint8_t scan_counter_enabled = 0;
    uint8_t speed_encoder_enabled = 0;
    uint8_t diagnostics_enabled = 0;

    uint16_t master_start_angle = 0;
    uint16_t master_end_angle = 0;
    uint16_t master_resolution = 0;
    uint16_t slave1_start_angle = 0;
    uint16_t slave1_end_angle = 0;
    uint16_t slave1_resolution = 0;
    uint16_t slave2_start_angle = 0;
    uint16_t slave2_end_angle = 0;
    uint16_t slave2_resolution = 0;
    uint16_t slave3_start_angle = 0;
    uint16_t slave3_end_angle = 0;
    uint16_t slave3_resolution = 0;

    start_msg *getMessage()
    {
        crc = crc32(crc, (const unsigned char *)this + 4, sizeof(start_msg) - 4);
        return this;
    }
}; // 58 bytes ??

struct end_msg
{
    uint32_t crc = 0x39FBEC28;
    uint32_t reserved[3] = {0};
    uint32_t op_code = 0x36;
}; // 20 bytes

struct reply_msg
{
    uint32_t crc;
    uint32_t reserved;
    uint32_t op_code;
    uint32_t res_code = 1; // Operation result. If the message is accepted, the returned value is 0x00. If the message is refused, the returned value is 0xEB
};                         // 16 bytes

struct optional_inf
{
    uint8_t header_id;
    uint16_t lenght;
    std::vector<uint8_t> payload;
};

struct monit_msg
{
    uint32_t dev_status;
    uint32_t op_code;
    uint32_t work_mode;
    uint32_t transaction_type;
    uint8_t scanner_id;
    uint16_t from_theta;
    uint16_t resolution;
    std::vector<optional_inf> optional;
};
#pragma pack(pop)
} // namespace Sentinel

#endif /* MSGS_DEFS_H_ */