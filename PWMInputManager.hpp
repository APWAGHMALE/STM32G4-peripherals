#include "PWMInputManager.hpp"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

bool PWMInputManager::running[CHANNEL_COUNT] = {false};
bool PWMInputManager::haveRising[CHANNEL_COUNT] = {false};
bool PWMInputManager::waitingForFalling[CHANNEL_COUNT] = {false};
bool PWMInputManager::valid[CHANNEL_COUNT] = {false};

uint32_t PWMInputManager::lastRising[CHANNEL_COUNT] = {0U};
uint32_t PWMInputManager::periodTicks[CHANNEL_COUNT] = {0U};
uint32_t PWMInputManager::highTicks[CHANNEL_COUNT] = {0U};
uint32_t PWMInputManager::counterClockHz[CHANNEL_COUNT] = {0U};
uint32_t PWMInputManager::lastCaptureTick[CHANNEL_COUNT] = {0U};


// --------------------------------------------------
// Enum validation
// --------------------------------------------------
uint8_t PWMInputManager::ToIndex(PWMInput input)
{
    return static_cast<uint8_t>(input);
}

bool PWMInputManager::IsValidInput(PWMInput input)
{
    return ToIndex(input) < CHANNEL_COUNT;
}


// --------------------------------------------------
// Get timer input clock
// TIM3: APB1
// TIM8: APB2
// --------------------------------------------------
uint32_t PWMInputManager::GetTimerClock(
    TIM_HandleTypeDef* timer)
{
    if (timer == nullptr)
    {
        return 0U;
    }

    RCC_ClkInitTypeDef clockConfig = {0};
    uint32_t flashLatency = 0U;

    HAL_RCC_GetClockConfig(&clockConfig, &flashLatency);

    if (timer->Instance == TIM3)
    {
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();

        return (clockConfig.APB1CLKDivider == RCC_HCLK_DIV1)
                   ? pclk
                   : (pclk * 2U);
    }

    if (timer->Instance == TIM8)
    {
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();

        return (clockConfig.APB2CLKDivider == RCC_HCLK_DIV1)
                   ? pclk
                   : (pclk * 2U);
    }

    return 0U;
}


// --------------------------------------------------
// Configure input timer for a 1 MHz counter clock
// ARR = 65535 gives a 65.536 ms counter range
// --------------------------------------------------
bool PWMInputManager::ConfigureTimer(
    TIM_HandleTypeDef* timer)
{
    if (timer == nullptr)
    {
        return false;
    }

    uint32_t timerClock = GetTimerClock(timer);

    if ((timerClock < COUNTER_TARGET_HZ) ||
        ((timerClock % COUNTER_TARGET_HZ) != 0U))
    {
        return false;
    }

    // Do not reconfigure a timer while it is running.
    __HAL_TIM_DISABLE(timer);

    uint32_t prescaler =
        (timerClock / COUNTER_TARGET_HZ) - 1U;

    __HAL_TIM_SET_PRESCALER(timer, prescaler);
    __HAL_TIM_SET_AUTORELOAD(timer, 0xFFFFU);
    __HAL_TIM_SET_COUNTER(timer, 0U);

    // Transfer PSC and ARR values into the active registers.
    __HAL_TIM_GENERATE_EVENT(timer, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(timer, 0U);

    return true;
}


// --------------------------------------------------
// Get actual counter clock for a specific input
// --------------------------------------------------
uint32_t PWMInputManager::GetCounterClock(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return 0U;
    }

    const PWMInputConfig& config = GetPWMInputConfig(input);

    if (config.timer == nullptr)
    {
        return 0U;
    }

    uint32_t timerClock = GetTimerClock(config.timer);
    uint32_t prescaler = config.timer->Instance->PSC;

    if (prescaler == 0xFFFFFFFFU)
    {
        return 0U;
    }

    return timerClock / (prescaler + 1U);
}


// --------------------------------------------------
// Initialize manager
// Call after MX_TIM3_Init() and MX_TIM8_Init()
// --------------------------------------------------
void PWMInputManager::Init()
{
    // Configure each shared timer only once.
    if (!ConfigureTimer(&htim3))
    {
        return;
    }

    if (!ConfigureTimer(&htim8))
    {
        return;
    }

    for (uint8_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        running[i] = false;
        haveRising[i] = false;
        waitingForFalling[i] = false;
        valid[i] = false;

        lastRising[i] = 0U;
        periodTicks[i] = 0U;
        highTicks[i] = 0U;
        lastCaptureTick[i] = 0U;

        counterClockHz[i] =
            GetCounterClock(static_cast<PWMInput>(i));
    }
}


// --------------------------------------------------
// Start one input channel
// --------------------------------------------------
bool PWMInputManager::Start(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return false;
    }

    uint8_t index = ToIndex(input);
    const PWMInputConfig& config = GetPWMInputConfig(input);

    if (config.timer == nullptr)
    {
        return false;
    }

    if (running[index])
    {
        return true;
    }

    // Start with rising-edge capture.
    __HAL_TIM_SET_CAPTUREPOLARITY(
        config.timer,
        config.channel,
        TIM_INPUTCHANNELPOLARITY_RISING);

    haveRising[index] = false;
    waitingForFalling[index] = false;
    valid[index] = false;

    lastRising[index] = 0U;
    periodTicks[index] = 0U;
    highTicks[index] = 0U;
    lastCaptureTick[index] = HAL_GetTick();

    counterClockHz[index] = GetCounterClock(input);

    if (counterClockHz[index] == 0U)
    {
        return false;
    }

    if (HAL_TIM_IC_Start_IT(
            config.timer,
            config.channel) != HAL_OK)
    {
        return false;
    }

    running[index] = true;
    return true;
}


// --------------------------------------------------
// Stop one input channel
// --------------------------------------------------
bool PWMInputManager::Stop(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return false;
    }

    uint8_t index = ToIndex(input);
    const PWMInputConfig& config = GetPWMInputConfig(input);

    if (HAL_TIM_IC_Stop_IT(
            config.timer,
            config.channel) != HAL_OK)
    {
        return false;
    }

    running[index] = false;
    valid[index] = false;
    haveRising[index] = false;
    waitingForFalling[index] = false;

    return true;
}


// --------------------------------------------------
// Start all channels
// --------------------------------------------------
void PWMInputManager::StartAll()
{
    for (uint8_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        (void)Start(static_cast<PWMInput>(i));
    }
}


// --------------------------------------------------
// Stop all channels
// --------------------------------------------------
void PWMInputManager::StopAll()
{
    for (uint8_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        if (running[i])
        {
            (void)Stop(static_cast<PWMInput>(i));
        }
    }
}


// --------------------------------------------------
// Handle one capture value
// --------------------------------------------------
void PWMInputManager::CaptureCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim == nullptr)
    {
        return;
    }

    for (uint8_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        PWMInput input = static_cast<PWMInput>(i);
        const PWMInputConfig& config = GetPWMInputConfig(input);

        if (!running[i] || (config.timer != htim))
        {
            continue;
        }

        uint32_t activeChannel = 0U;

        switch (config.channel)
        {
            case TIM_CHANNEL_1:
                activeChannel = HAL_TIM_ACTIVE_CHANNEL_1;
                break;

            case TIM_CHANNEL_2:
                activeChannel = HAL_TIM_ACTIVE_CHANNEL_2;
                break;

            case TIM_CHANNEL_3:
                activeChannel = HAL_TIM_ACTIVE_CHANNEL_3;
                break;

            case TIM_CHANNEL_4:
                activeChannel = HAL_TIM_ACTIVE_CHANNEL_4;
                break;

            default:
                continue;
        }

        if (htim->Channel != activeChannel)
        {
            continue;
        }

        uint32_t capture = HAL_TIM_ReadCapturedValue(
            htim, config.channel);

        uint32_t counterPeriod =
            __HAL_TIM_GET_AUTORELOAD(htim) + 1U;

        lastCaptureTick[i] = HAL_GetTick();

        if (!waitingForFalling[i])
        {
            // Rising edge
            if (haveRising[i])
            {
                uint32_t measuredPeriod = CalculateDelta(
                    capture,
                    lastRising[i],
                    counterPeriod);

                if ((measuredPeriod > 0U) &&
                    (highTicks[i] > 0U) &&
                    (highTicks[i] <= measuredPeriod))
                {
                    periodTicks[i] = measuredPeriod;
                    valid[i] = true;
                }
            }

            lastRising[i] = capture;
            haveRising[i] = true;
            waitingForFalling[i] = true;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                config.channel,
                TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            // Falling edge: measure the high pulse width.
            highTicks[i] = CalculateDelta(
                capture,
                lastRising[i],
                counterPeriod);

            waitingForFalling[i] = false;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                config.channel,
                TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}


// --------------------------------------------------
// Calculate counter delta, including one rollover
// --------------------------------------------------
uint32_t PWMInputManager::CalculateDelta(
    uint32_t current,
    uint32_t previous,
    uint32_t counterPeriod)
{
    if (current >= previous)
    {
        return current - previous;
    }

    return (counterPeriod - previous) + current;
}


// --------------------------------------------------
// Frequency in Hz
// --------------------------------------------------
uint32_t PWMInputManager::GetFrequency(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return 0U;
    }

    uint8_t index = ToIndex(input);

    if (!valid[index] || (periodTicks[index] == 0U))
    {
        return 0U;
    }

    uint32_t clockHz = counterClockHz[index];

    if (clockHz == 0U)
    {
        return 0U;
    }

    return clockHz / periodTicks[index];
}


// --------------------------------------------------
// Duty cycle in percent
// --------------------------------------------------
float PWMInputManager::GetDuty(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return 0.0f;
    }

    uint8_t index = ToIndex(input);

    if (!valid[index] || (periodTicks[index] == 0U))
    {
        return 0.0f;
    }

    return ((float)highTicks[index] * 100.0f) /
           (float)periodTicks[index];
}


// --------------------------------------------------
// Period in microseconds
// --------------------------------------------------
uint32_t PWMInputManager::GetPeriodUs(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return 0U;
    }

    uint8_t index = ToIndex(input);

    if (!valid[index] ||
        (periodTicks[index] == 0U) ||
        (counterClockHz[index] == 0U))
    {
        return 0U;
    }

    return (uint32_t)(
        ((uint64_t)periodTicks[index] * 1000000ULL) /
        counterClockHz[index]);
}


// --------------------------------------------------
// High pulse width in microseconds
// --------------------------------------------------
uint32_t PWMInputManager::GetPulseWidthUs(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return 0U;
    }

    uint8_t index = ToIndex(input);

    if (!valid[index] ||
        (counterClockHz[index] == 0U))
    {
        return 0U;
    }

    return (uint32_t)(
        ((uint64_t)highTicks[index] * 1000000ULL) /
        counterClockHz[index]);
}


// --------------------------------------------------
// Valid measurement status
// --------------------------------------------------
bool PWMInputManager::IsValid(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return false;
    }

    return valid[ToIndex(input)];
}


// --------------------------------------------------
// Running status
// --------------------------------------------------
bool PWMInputManager::IsRunning(PWMInput input)
{
    if (!IsValidInput(input))
    {
        return false;
    }

    return running[ToIndex(input)];
}


// --------------------------------------------------
// Expire stale readings
// Call periodically from the main loop
// --------------------------------------------------
void PWMInputManager::Update()
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        if (running[i] &&
            ((uint32_t)(now - lastCaptureTick[i]) >
             SIGNAL_TIMEOUT_MS))
        {
            valid[i] = false;
            haveRising[i] = false;
            waitingForFalling[i] = false;
        }
    }
}
