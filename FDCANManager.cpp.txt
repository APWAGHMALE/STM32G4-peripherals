
#include "FDCANManager.hpp"

bool FDCANManager::running[2] = {false, false};
uint32_t FDCANManager::bitrateHz[2] = {0U, 0U};
bool FDCANManager::initialized = false;


// ==================================================
// VALIDATION
// ==================================================

bool FDCANManager::IsValidChannel(CANChannel channel)
{
    return channel == CANChannel::CAN1 ||
           channel == CANChannel::CAN2;
}


// ==================================================
// FILTER CONFIGURATION
// Accept standard and extended data frames in FIFO0.
// Reject remote frames.
// ==================================================

bool FDCANManager::ConfigureFilter(CANChannel channel)
{
    if (!IsValidChannel(channel))
        return false;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    return HAL_FDCAN_ConfigGlobalFilter(
        fdcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    ) == HAL_OK;
}


// ==================================================
// INITIALIZATION
// Call after MX_FDCAN1_Init() and MX_FDCAN2_Init().
// ==================================================

bool FDCANManager::Init()
{
    initialized = false;

    for (uint8_t i = 0U; i < 2U; i++)
    {
        running[i] = false;
        bitrateHz[i] = 0U;
    }

    if (GetCANConfig(CANChannel::CAN1).fdcan == nullptr ||
        GetCANConfig(CANChannel::CAN2).fdcan == nullptr)
    {
        return false;
    }

    if (!ConfigureFilter(CANChannel::CAN1))
        return false;

    if (!ConfigureFilter(CANChannel::CAN2))
        return false;

    initialized = true;
    return true;
}


// ==================================================
// CALCULATE NOMINAL BIT TIMING
//
// Assumes FDCAN ClockDivider = FDCAN_CLOCK_DIV1.
// Selects timing near an 80% sample point.
// ==================================================

bool FDCANManager::CalculateTiming(
    uint32_t clockHz,
    uint32_t bitrate,
    uint32_t& prescaler,
    uint32_t& timeSeg1,
    uint32_t& timeSeg2,
    uint32_t& sjw)
{
    if (clockHz == 0U || bitrate == 0U)
        return false;

    uint64_t bestError = UINT64_MAX;
    uint32_t bestPrescaler = 0U;
    uint32_t bestSeg1 = 0U;
    uint32_t bestSeg2 = 0U;
    uint32_t bestSJW = 0U;

    // STM32G4 FDCAN nominal timing ranges.
    for (uint32_t p = 1U; p <= 512U; p++)
    {
        uint64_t denominator = (uint64_t)p * bitrate;

        uint32_t totalTQ = (uint32_t)(
            ((uint64_t)clockHz + denominator / 2ULL)
            / denominator
        );

        // Total TQ = SyncSeg + TimeSeg1 + TimeSeg2.
        if (totalTQ < 4U || totalTQ > 385U)
            continue;

        uint32_t seg1 =
            ((totalTQ * 80U + 50U) / 100U);

        if (seg1 > 0U)
            seg1--;

        if (seg1 < 1U)
            seg1 = 1U;

        if (seg1 > 256U)
            continue;

        uint32_t seg2 = totalTQ - 1U - seg1;

        if (seg2 < 1U || seg2 > 128U)
            continue;

        uint32_t candidateSJW =
            (seg2 < 4U) ? seg2 : 4U;

        uint64_t actualDenominator =
            (uint64_t)p * totalTQ;

        uint64_t actualBitrate =
            ((uint64_t)clockHz + actualDenominator / 2ULL)
            / actualDenominator;

        uint64_t error =
            (actualBitrate > bitrate)
            ? actualBitrate - bitrate
            : bitrate - actualBitrate;

        if (error < bestError)
        {
            bestError = error;
            bestPrescaler = p;
            bestSeg1 = seg1;
            bestSeg2 = seg2;
            bestSJW = candidateSJW;

            if (error == 0U)
                break;
        }
    }

    if (bestPrescaler == 0U)
        return false;

    // Reject a timing solution with more than 1% bitrate error.
    if (bestError * 100ULL > bitrate)
        return false;

    prescaler = bestPrescaler;
    timeSeg1 = bestSeg1;
    timeSeg2 = bestSeg2;
    sjw = bestSJW;

    return true;
}


// ==================================================
// SET BITRATE
//
// Can be called while stopped or running.
// If running, the controller is stopped and restarted.
// ==================================================

bool FDCANManager::SetBitrate(
    CANChannel channel,
    uint32_t bitrate)
{
    if (!initialized || !IsValidChannel(channel))
        return false;

    if (bitrate == 0U)
        return false;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    // This implementation uses nominal timing for
    // classical CAN and requires a divide-by-1 clock.
    if (fdcan->Init.ClockDivider != FDCAN_CLOCK_DIV1)
        return false;

    uint32_t fdcanClock =
        HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    uint32_t prescaler = 0U;
    uint32_t seg1 = 0U;
    uint32_t seg2 = 0U;
    uint32_t sjw = 0U;

    if (!CalculateTiming(
            fdcanClock,
            bitrate,
            prescaler,
            seg1,
            seg2,
            sjw))
    {
        return false;
    }

    bool wasRunning =
        running[static_cast<uint8_t>(channel)];

    if (wasRunning)
    {
        if (HAL_FDCAN_Stop(fdcan) != HAL_OK)
            return false;

        running[static_cast<uint8_t>(channel)] = false;
    }

    // Deinitialize and reinitialize with the new timing.
    if (HAL_FDCAN_DeInit(fdcan) != HAL_OK)
        return false;

    fdcan->Init.NominalPrescaler = prescaler;
    fdcan->Init.NominalTimeSeg1 = seg1;
    fdcan->Init.NominalTimeSeg2 = seg2;
    fdcan->Init.NominalSyncJumpWidth = sjw;

    if (HAL_FDCAN_Init(fdcan) != HAL_OK)
        return false;

    if (!ConfigureFilter(channel))
        return false;

    uint8_t index = static_cast<uint8_t>(channel);
    bitrateHz[index] = bitrate;

    if (wasRunning)
    {
        if (HAL_FDCAN_Start(fdcan) != HAL_OK)
            return false;

        running[index] = true;
    }

    return true;
}


// ==================================================
// GET CONFIGURED NOMINAL BITRATE
// ==================================================

uint32_t FDCANManager::GetBitrate(CANChannel channel)
{
    if (!IsValidChannel(channel))
        return 0U;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return 0U;

    uint64_t clockHz =
        HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

    if (fdcan->Init.ClockDivider != FDCAN_CLOCK_DIV1)
        return 0U;

    uint64_t prescaler =
        (uint64_t)fdcan->Init.NominalPrescaler;

    uint64_t totalTQ =
        1ULL +
        fdcan->Init.NominalTimeSeg1 +
        fdcan->Init.NominalTimeSeg2;

    uint64_t denominator = prescaler * totalTQ;

    if (clockHz == 0ULL || denominator == 0ULL)
        return 0U;

    return (uint32_t)(
        (clockHz + denominator / 2ULL) / denominator
    );
}


// ==================================================
// START / STOP
// ==================================================

bool FDCANManager::Start(CANChannel channel)
{
    if (!initialized || !IsValidChannel(channel))
        return false;

    uint8_t index = static_cast<uint8_t>(channel);

    if (running[index])
        return true;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    if (HAL_FDCAN_Start(fdcan) != HAL_OK)
        return false;

    running[index] = true;
    return true;
}


bool FDCANManager::Stop(CANChannel channel)
{
    if (!IsValidChannel(channel))
        return false;

    uint8_t index = static_cast<uint8_t>(channel);

    if (!running[index])
        return true;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    if (HAL_FDCAN_Stop(fdcan) != HAL_OK)
        return false;

    running[index] = false;
    return true;
}


bool FDCANManager::IsRunning(CANChannel channel)
{
    if (!IsValidChannel(channel))
        return false;

    return running[static_cast<uint8_t>(channel)];
}


// ==================================================
// CAN DATA LENGTH CODE
// Classical CAN supports 0 to 8 bytes.
// ==================================================

uint32_t FDCANManager::LengthToDLC(uint8_t length)
{
    switch (length)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return 0xFFFFFFFFU;
    }
}


uint8_t FDCANManager::DLCToLength(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}


// ==================================================
// TRANSMIT CLASSICAL CAN FRAME
// ==================================================

bool FDCANManager::Transmit(
    CANChannel channel,
    const FDCANMessage& message)
{
    if (!IsValidChannel(channel) || !IsRunning(channel))
        return false;

    if (message.length > 8U)
        return false;

    if ((!message.extended && message.id > 0x7FFU) ||
        (message.extended && message.id > 0x1FFFFFFFU))
    {
        return false;
    }

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) == 0U)
        return false;

    FDCAN_TxHeaderTypeDef header = {};

    header.Identifier = message.id;
    header.IdType = message.extended
        ? FDCAN_EXTENDED_ID
        : FDCAN_STANDARD_ID;

    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = LengthToDLC(message.length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        fdcan,
        &header,
        const_cast<uint8_t*>(message.data)
    ) == HAL_OK;
}


// ==================================================
// RECEIVE CLASSICAL CAN FRAME FROM FIFO0
// ==================================================

bool FDCANManager::Receive(
    CANChannel channel,
    FDCANMessage& message)
{
    if (!IsValidChannel(channel) || !IsRunning(channel))
        return false;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return false;

    if (HAL_FDCAN_GetRxFifoFillLevel(
            fdcan, FDCAN_RX_FIFO0) == 0U)
    {
        return false;
    }

    FDCAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {0};

    if (HAL_FDCAN_GetRxMessage(
            fdcan,
            FDCAN_RX_FIFO0,
            &header,
            data) != HAL_OK)
    {
        return false;
    }

    // This manager receives classical CAN frames only.
    if (header.FDFormat != FDCAN_CLASSIC_CAN ||
        header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return false;
    }

    uint8_t length = DLCToLength(header.DataLength);

    message.id = header.Identifier;
    message.extended = (header.IdType == FDCAN_EXTENDED_ID);
    message.length = length;

    for (uint8_t i = 0U; i < 8U; i++)
        message.data[i] = (i < length) ? data[i] : 0U;

    return true;
}


// ==================================================
// ERROR REPORTING
// ==================================================

uint32_t FDCANManager::GetLastError(CANChannel channel)
{
    if (!IsValidChannel(channel))
        return 0U;

    FDCAN_HandleTypeDef* fdcan =
        GetCANConfig(channel).fdcan;

    if (fdcan == nullptr)
        return 0U;

    return HAL_FDCAN_GetError(fdcan);
}
