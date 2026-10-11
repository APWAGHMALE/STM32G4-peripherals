
#ifndef CAN_FRAMES_HPP
#define CAN_FRAMES_HPP

#include <stdint.h>

/*
 * HIL Controller CAN Frame Definitions
 *
 * Classical CAN:
 * Standard 11-bit identifiers
 * Maximum payload: 8 bytes
 *
 * Multi-byte values are transmitted little-endian.
 */

namespace CANFrames
{
    /* CAN frame IDs */
    constexpr uint32_t ADC_DATA       = 0x100;
    constexpr uint32_t PWM_INPUT_DATA = 0x110;
    constexpr uint32_t PWM_OUTPUT_CMD = 0x120;
    constexpr uint32_t PWM_OUTPUT_STA = 0x121;
    constexpr uint32_t DIGITAL_IO     = 0x130;
    constexpr uint32_t SYSTEM_STATUS  = 0x140;

    constexpr uint8_t ADC_CHANNEL_COUNT = 8;
    constexpr uint8_t PWM_CHANNEL_COUNT = 8;

    constexpr uint8_t ADC_CHANNELS_PER_FRAME = 4;
    constexpr uint8_t PWM_CHANNELS_PER_FRAME = 2;

    struct Frame
    {
        uint32_t id;
        uint8_t data[8];
        uint8_t length;
    };

    /*
     * ADC_DATA frame
     *
     * Bytes 0-1: ADC channel 1 raw value
     * Bytes 2-3: ADC channel 2 raw value
     * Bytes 4-5: ADC channel 3 raw value
     * Bytes 6-7: ADC channel 4 raw value
     *
     * ADC channels 5-8 use the same layout
     * in a second frame with ID ADC_DATA + 1.
     */

    /*
     * PWM_INPUT_DATA frame
     *
     * Bytes 0-1: PWM frequency, channel 1 (Hz)
     * Bytes 2-3: PWM duty cycle, channel 1 (0.01%)
     * Bytes 4-5: PWM frequency, channel 2 (Hz)
     * Bytes 6-7: PWM duty cycle, channel 2 (0.01%)
     *
     * Frequency is limited to 65535 Hz.
     * Duty cycle is represented from 0 to 10000.
     */

    /*
     * PWM_OUTPUT_CMD frame
     *
     * Bytes 0-1: channel number (1-12)
     * Bytes 2-5: frequency in Hz (uint32)
     * Bytes 6-7: duty cycle (0-10000, representing 0-100.00%)
     */

    /*
     * PWM_OUTPUT_STA frame
     *
     * Byte 0: channel number (1-12)
     * Byte 1: running status (0 = stopped, 1 = running)
     * Bytes 2-3: actual frequency in Hz, uint16
     * Bytes 4-5: duty cycle in 0.01%, uint16
     * Bytes 6-7: reserved
     */

    /*
     * DIGITAL_IO frame
     *
     * Bytes 0-3: digital input bitmask
     * Bytes 4-7: digital output bitmask
     */

    /*
     * SYSTEM_STATUS frame
     *
     * Byte 0: system status
     * Byte 1: error category
     * Bytes 2-3: error code
     * Bytes 4-7: reserved
     */
}

#endif
