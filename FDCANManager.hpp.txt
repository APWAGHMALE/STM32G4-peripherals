
#ifndef FDCAN_MANAGER_HPP
#define FDCAN_MANAGER_HPP

#include "main.h"
#include "PinMap.h"
#include <stdint.h>

struct FDCANMessage
{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    bool extended;
};

class FDCANManager
{
public:
    static bool Init();

    static bool Start(CANChannel channel);
    static bool Stop(CANChannel channel);

    // Set nominal bitrate in bits per second.
    // Example: 250000, 500000, 1000000.
    static bool SetBitrate(
        CANChannel channel,
        uint32_t bitrate
    );

    static uint32_t GetBitrate(CANChannel channel);

    static bool Transmit(
        CANChannel channel,
        const FDCANMessage& message
    );

    // Poll FIFO0 for a received classical CAN frame.
    static bool Receive(
        CANChannel channel,
        FDCANMessage& message
    );

    static uint32_t GetLastError(CANChannel channel);
    static bool IsRunning(CANChannel channel);

private:
    static bool IsValidChannel(CANChannel channel);

    static bool ConfigureFilter(CANChannel channel);

    static bool CalculateTiming(
        uint32_t clockHz,
        uint32_t bitrate,
        uint32_t& prescaler,
        uint32_t& timeSeg1,
        uint32_t& timeSeg2,
        uint32_t& sjw
    );

    static uint32_t LengthToDLC(uint8_t length);
    static uint8_t DLCToLength(uint32_t dlc);

    static bool running[2];
    static uint32_t bitrateHz[2];
    static bool initialized;
};

#endif
