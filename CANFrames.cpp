
#include "CANFrames.hpp"

namespace CANFrames
{
    static void WriteU16(uint8_t* data, uint8_t offset,
                         uint16_t value)
    {
        data[offset]     = static_cast<uint8_t>(value & 0xFFU);
        data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    }

    static uint16_t ReadU16(const uint8_t* data, uint8_t offset)
    {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(data[offset]) |
            (static_cast<uint16_t>(data[offset + 1]) << 8)
        );
    }

    static void WriteU32(uint8_t* data, uint8_t offset,
                         uint32_t value)
    {
        data[offset]     = static_cast<uint8_t>(value & 0xFFU);
        data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
        data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
        data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
    }

    static uint32_t ReadU32(const uint8_t* data, uint8_t offset)
    {
        return
            static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24);
    }
}
