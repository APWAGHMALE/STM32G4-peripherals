
#include "PWMInputManager.hpp"

PWMInputManager::CaptureState PWMInputManager::state[8] = {};


// ==================================================
// CHANNEL VALIDATION
// ==================================================

bool PWMInputManager::IsValidChannel(uint8_t channel)
{
    return (channel >= 1U && channel <= 8U);
}


// ==================================================
// TIMER CLOCK
// ==================================================

uint32_t PWMInputManager::GetTimerClock(
    TIM_HandleTypeDef* timer)
{
    if (timer == nullptr)
        return 0U;

    RCC_ClkInitTypeDef clockConfig = {};
    uint32_t flashLatency = 0U;

    HAL_RCC_GetClockConfig(&clockConfig, &flashLatency);

    uint32_t pclk;
    uint32_t apbDivider;

    if (timer->Instance == TIM3)
    {
        // TIM3 uses APB1.
        pclk = HAL_RCC_GetPCLK1Freq();
        apbDivider = clockConfig.APB1CLKDivider;
    }
    else if (timer->Instance == TIM8 ||
             timer->Instance == TIM15)
    {
        // TIM8 and TIM15 use APB2.
        pclk = HAL_RCC_GetPCLK2Freq();
        apbDivider = clockConfig.APB2CLKDivider;
    }
    else
    {
        return 0U;
    }

    // STM32G491 timer clock:
    // APB divider = 1: timer clock = PCLK
    // APB divider > 1: timer clock = 2 * PCLK
    if (apbDivider != RCC_HCLK_DIV1)
    {
        pclk *= 2U;
    }

    return pclk;
}


uint32_t PWMInputManager::GetCounterClock(
    TIM_HandleTypeDef* timer)
{
    if (timer == nullptr)
        return 0U;

    uint32_t timerClock = GetTimerClock(timer);

    uint32_t prescaler =
        __HAL_TIM_GET_PRESCALER(timer) + 1U;

    if (prescaler == 0U)
        return 0U;

    return timerClock / prescaler;
}


// ==================================================
// CAPTURE HELPERS
// ==================================================

uint32_t PWMInputManager::GetElapsedTicks(
    TIM_HandleTypeDef* timer,
    uint32_t start,
    uint32_t end)
{
    uint64_t modulus =
        (uint64_t)__HAL_TIM_GET_AUTORELOAD(timer) + 1ULL;

    return (uint32_t)(
        ((uint64_t)end + modulus - start) % modulus
    );
}


uint32_t PWMInputManager::GetTimerChannel(
    uint32_t activeChannel)
{
    switch (activeChannel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            return TIM_CHANNEL_1;

        case HAL_TIM_ACTIVE_CHANNEL_2:
            return TIM_CHANNEL_2;

        case HAL_TIM_ACTIVE_CHANNEL_3:
            return TIM_CHANNEL_3;

        case HAL_TIM_ACTIVE_CHANNEL_4:
            return TIM_CHANNEL_4;

        default:
            return 0U;
    }
}


// ==================================================
// INITIALIZATION
// ==================================================

bool PWMInputManager::Init()
{
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (pwmInputMap[i].timer == nullptr)
            return false;

        state[i].lastRising = 0U;
        state[i].highTicks = 0U;

        state[i].frequencyHz = 0U;
        state[i].periodUs = 0U;
        state[i].pulseWidthUs = 0U;
        state[i].dutyPercent = 0.0f;

        state[i].haveRising = false;
        state[i].haveHigh = false;
        state[i].waitingForFalling = false;
        state[i].running = false;
        state[i].valid = false;
    }

    return true;
}


// ==================================================
// START ONE INPUT
// ==================================================

bool PWMInputManager::Start(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return false;

    uint8_t index = channel - 1U;

    const PWMInputConfig& config = pwmInputMap[index];

    if (config.timer == nullptr)
        return false;

    if (state[index].running)
        return true;

    if (GetCounterClock(config.timer) == 0U)
        return false;

    // Reset measurement state.
    state[index].lastRising = 0U;
    state[index].highTicks = 0U;

    state[index].frequencyHz = 0U;
    state[index].periodUs = 0U;
    state[index].pulseWidthUs = 0U;
    state[index].dutyPercent = 0.0f;

    state[index].haveRising = false;
    state[index].haveHigh = false;
    state[index].waitingForFalling = false;
    state[index].valid = false;

    // First capture a rising edge.
    __HAL_TIM_SET_CAPTUREPOLARITY(
        config.timer,
        config.channel,
        TIM_INPUTCHANNELPOLARITY_RISING
    );

    if (HAL_TIM_IC_Start_IT(
            config.timer,
            config.channel) != HAL_OK)
    {
        return false;
    }

    state[index].running = true;

    return true;
}


// ==================================================
// STOP ONE INPUT
// ==================================================

bool PWMInputManager::Stop(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return false;

    uint8_t index = channel - 1U;

    const PWMInputConfig& config = pwmInputMap[index];

    if (!state[index].running)
        return true;

    if (HAL_TIM_IC_Stop_IT(
            config.timer,
            config.channel) != HAL_OK)
    {
        return false;
    }

    __HAL_TIM_SET_CAPTUREPOLARITY(
        config.timer,
        config.channel,
        TIM_INPUTCHANNELPOLARITY_RISING
    );

    state[index].running = false;
    state[index].valid = false;

    return true;
}


// ==================================================
// START ALL INPUTS
// ==================================================

bool PWMInputManager::StartAll()
{
    for (uint8_t channel = 1U; channel <= 8U; channel++)
    {
        if (!Start(channel))
            return false;
    }

    return true;
}


// ==================================================
// STOP ALL INPUTS
// ==================================================

bool PWMInputManager::StopAll()
{
    bool success = true;

    for (uint8_t channel = 1U; channel <= 8U; channel++)
    {
        if (!Stop(channel))
            success = false;
    }

    return success;
}


// ==================================================
// INPUT CAPTURE HANDLER
// ==================================================

void PWMInputManager::HandleCapture(
    TIM_HandleTypeDef* htim)
{
    if (htim == nullptr)
        return;

    uint32_t activeChannel =
        GetTimerChannel(htim->Channel);

    if (activeChannel == 0U)
        return;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        const PWMInputConfig& config = pwmInputMap[i];

        if (config.timer != htim ||
            config.channel != activeChannel ||
            !state[i].running)
        {
            continue;
        }

        uint32_t capture =
            HAL_TIM_ReadCapturedValue(htim, activeChannel);

        uint32_t counterClock = GetCounterClock(htim);

        if (counterClock == 0U)
            return;

        if (!state[i].waitingForFalling)
        {
            // --------------------------------------
            // RISING EDGE
            // --------------------------------------

            if (state[i].haveRising)
            {
                uint32_t period = GetElapsedTicks(
                    htim,
                    state[i].lastRising,
                    capture
                );

                if (period > 0U)
                {
                    uint64_t periodUs =
                        ((uint64_t)period * 1000000ULL)
                        / counterClock;

                    state[i].periodUs =
                        periodUs > UINT32_MAX
                        ? UINT32_MAX
                        : (uint32_t)periodUs;

                    state[i].frequencyHz =
                        (uint32_t)(
                            ((uint64_t)counterClock + period / 2U)
                            / period
                        );

                    if (state[i].haveHigh &&
                        state[i].highTicks <= period)
                    {
                        state[i].dutyPercent =
                            100.0f *
                            (float)state[i].highTicks /
                            (float)period;

                        uint64_t pulseUs =
                            ((uint64_t)state[i].highTicks
                             * 1000000ULL)
                            / counterClock;

                        state[i].pulseWidthUs =
                            pulseUs > UINT32_MAX
                            ? UINT32_MAX
                            : (uint32_t)pulseUs;

                        state[i].valid = true;
                    }
                }
            }

            state[i].lastRising = capture;
            state[i].haveRising = true;
            state[i].waitingForFalling = true;

            // Switch this channel to falling-edge capture.
            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                activeChannel,
                TIM_INPUTCHANNELPOLARITY_FALLING
            );
        }
        else
        {
            // --------------------------------------
            // FALLING EDGE
            // --------------------------------------

            if (state[i].haveRising)
            {
                state[i].highTicks = GetElapsedTicks(
                    htim,
                    state[i].lastRising,
                    capture
                );

                state[i].haveHigh = true;
            }

            state[i].waitingForFalling = false;

            // Switch this channel back to rising-edge capture.
            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                activeChannel,
                TIM_INPUTCHANNELPOLARITY_RISING
            );
        }

        return;
    }
}


// ==================================================
// READ FREQUENCY
// ==================================================

uint32_t PWMInputManager::GetFrequency(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return 0U;

    return state[channel - 1U].frequencyHz;
}


// ==================================================
// READ DUTY CYCLE
// ==================================================

float PWMInputManager::GetDuty(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return 0.0f;

    return state[channel - 1U].dutyPercent;
}


// ==================================================
// READ PERIOD
// ==================================================

uint32_t PWMInputManager::GetPeriodUs(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return 0U;

    return state[channel - 1U].periodUs;
}


// ==================================================
// READ HIGH-PULSE WIDTH
// ==================================================

uint32_t PWMInputManager::GetPulseWidthUs(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return 0U;

    return state[channel - 1U].pulseWidthUs;
}


// ==================================================
// STATUS
// ==================================================

bool PWMInputManager::IsRunning(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return false;

    return state[channel - 1U].running;
}


bool PWMInputManager::IsValid(uint8_t channel)
{
    if (!IsValidChannel(channel))
        return false;

    return state[channel - 1U].valid;
}


// ==================================================
// HAL INPUT CAPTURE CALLBACK
// ==================================================

// If this callback already exists in your project,
// add the HandleCapture() call to the existing callback
// instead of defining this function a second time.

extern "C" void HAL_TIM_IC_CaptureCallback(
    TIM_HandleTypeDef* htim)
{
    PWMInputManager::HandleCapture(htim);
}
