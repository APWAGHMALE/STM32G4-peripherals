#include "PWMOutputManager.hpp"

constexpr uint8_t PWMOutputManager::OUTPUT_COUNT;

bool PWMOutputManager::running[OUTPUT_COUNT] = {false};
float PWMOutputManager::dutyPercent[OUTPUT_COUNT] = {0.0f};


//uint32_t prescaler = config.timer->Instance->PSC; 

// --------------------------------------------------
// Convert enum to array index
// --------------------------------------------------
uint8_t PWMOutputManager::ToIndex(PWMOutput output)
{
    return static_cast<uint8_t>(output);
}


// --------------------------------------------------
// Validate output
// --------------------------------------------------
bool PWMOutputManager::IsValidOutput(PWMOutput output)
{
    return ToIndex(output) < OUTPUT_COUNT;
}


// --------------------------------------------------
// Initialize manager
// Call after CubeMX timer initialization
// --------------------------------------------------
void PWMOutputManager::Init()
{
    for (uint8_t i = 0U; i < OUTPUT_COUNT; i++)
    {
        PWMOutput output = static_cast<PWMOutput>(i);
        const PWMOutputConfig& config = GetPWMOutputConfig(output);

        running[i] = false;

        uint32_t arr =
            __HAL_TIM_GET_AUTORELOAD(config.timer);

        uint32_t compare =
            __HAL_TIM_GET_COMPARE(config.timer, config.channel);

        uint32_t periodCounts = arr + 1U;

        if (periodCounts > 0U)
        {
            dutyPercent[i] =
                ((float)compare * 100.0f) /
                (float)periodCounts;

            if (dutyPercent[i] > 100.0f)
            {
                dutyPercent[i] = 100.0f;
            }
        }
        else
        {
            dutyPercent[i] = 0.0f;
        }
    }
}


// --------------------------------------------------
// Get timer input clock using HAL functions
// STM32G491RE timer clock mapping used by this project
// --------------------------------------------------
uint32_t PWMOutputManager::GetTimerClock(
    TIM_HandleTypeDef* timer)
{
    if (timer == nullptr)
    {
        return 0U;
    }

    RCC_ClkInitTypeDef clockConfig = {0};
    uint32_t flashLatency = 0U;

    HAL_RCC_GetClockConfig(&clockConfig, &flashLatency);

    // APB2 timers
    if ((timer->Instance == TIM1)  ||
        (timer->Instance == TIM8)  ||
        (timer->Instance == TIM15))
    {
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

        if (clockConfig.APB2CLKDivider == RCC_HCLK_DIV1)
        {
            return pclk2;
        }

        return pclk2 * 2U;
    }

    // APB1 timers
    if ((timer->Instance == TIM2) ||
        (timer->Instance == TIM3) ||
        (timer->Instance == TIM4))
    {
        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

        if (clockConfig.APB1CLKDivider == RCC_HCLK_DIV1)
        {
            return pclk1;
        }

        return pclk1 * 2U;
    }

    return 0U;
}


// --------------------------------------------------
// Start one PWM output
// --------------------------------------------------
bool PWMOutputManager::Start(PWMOutput output)
{
    if (!IsValidOutput(output))
    {
        return false;
    }

    uint8_t index = ToIndex(output);
    const PWMOutputConfig& config = GetPWMOutputConfig(output);

    if (HAL_TIM_PWM_Start(
            config.timer,
            config.channel) == HAL_OK)
    {
        running[index] = true;
        return true;
    }

    return false;
}


// --------------------------------------------------
// Stop one PWM output
// --------------------------------------------------
bool PWMOutputManager::Stop(PWMOutput output)
{
    if (!IsValidOutput(output))
    {
        return false;
    }

    uint8_t index = ToIndex(output);
    const PWMOutputConfig& config = GetPWMOutputConfig(output);

    if (HAL_TIM_PWM_Stop(
            config.timer,
            config.channel) == HAL_OK)
    {
        running[index] = false;
        return true;
    }

    return false;
}


// --------------------------------------------------
// Start all PWM outputs
// --------------------------------------------------
void PWMOutputManager::StartAll()
{
    for (uint8_t i = 0U; i < OUTPUT_COUNT; i++)
    {
        Start(static_cast<PWMOutput>(i));
    }
}


// --------------------------------------------------
// Stop all PWM outputs
// --------------------------------------------------
void PWMOutputManager::StopAll()
{
    for (uint8_t i = 0U; i < OUTPUT_COUNT; i++)
    {
        Stop(static_cast<PWMOutput>(i));
    }
}


// --------------------------------------------------
// Set duty cycle for one output
// --------------------------------------------------
bool PWMOutputManager::SetDuty(
    PWMOutput output,
    float duty)
{
    if (!IsValidOutput(output))
    {
        return false;
    }

    if ((duty < 0.0f) || (duty > 100.0f))
    {
        return false;
    }

    uint8_t index = ToIndex(output);

    dutyPercent[index] = duty;

    return ApplyDuty(index);
}


// --------------------------------------------------
// Apply the stored duty cycle to the timer CCR register
// --------------------------------------------------
bool PWMOutputManager::ApplyDuty(uint8_t index)
{
    if (index >= OUTPUT_COUNT)
    {
        return false;
    }

    PWMOutput output = static_cast<PWMOutput>(index);
    const PWMOutputConfig& config = GetPWMOutputConfig(output);

    uint32_t arr =
        __HAL_TIM_GET_AUTORELOAD(config.timer);

    uint64_t periodCounts = (uint64_t)arr + 1ULL;

    uint64_t compare =
        (uint64_t)(
            (periodCounts * dutyPercent[index] + 50.0f)
            / 100.0f
        );

    // Allow ARR + 1 except when it exceeds the
    // 16-bit compare-register range.
    uint32_t maxCompare =
        (arr < 0xFFFFU) ? (arr + 1U) : arr;

    if (compare > maxCompare)
    {
        compare = maxCompare;
    }

    __HAL_TIM_SET_COMPARE(
        config.timer,
        config.channel,
        (uint32_t)compare
    );

    return true;
}


// --------------------------------------------------
// Set PWM frequency
// Chooses PSC and ARR based on the timer input clock
// --------------------------------------------------
bool PWMOutputManager::SetFrequency(
    PWMOutput output,
    uint32_t frequencyHz)
{
    if (!IsValidOutput(output) || (frequencyHz == 0U))
    {
        return false;
    }

    const PWMOutputConfig& config = GetPWMOutputConfig(output);

    TIM_HandleTypeDef* timer = config.timer;

    uint32_t timerClock = GetTimerClock(timer);

    if (timerClock == 0U)
    {
        return false;
    }

    uint32_t prescaler = 0U;
    uint64_t periodCounts = 0ULL;

    // Use a 16-bit ARR range for all mapped timers.
    // Choose the smallest prescaler that fits.
    while (true)
    {
        uint64_t denominator =
            ((uint64_t)prescaler + 1ULL) * frequencyHz;

        periodCounts =
            ((uint64_t)timerClock + denominator / 2ULL)
            / denominator;

        // Require at least two counter ticks per period.
        if (periodCounts < 2ULL)
        {
            return false;
        }

        if (periodCounts <= 65536ULL)
        {
            break;
        }

        if (prescaler >= 65535U)
        {
            return false;
        }

        prescaler++;
    }

    uint32_t arr = (uint32_t)(periodCounts - 1ULL);

    // Program the timer registers.
    __HAL_TIM_SET_PRESCALER(timer, prescaler);
    __HAL_TIM_SET_AUTORELOAD(timer, arr);

    // Preserve the duty cycle for every mapped output
    // that shares this timer.
    for (uint8_t i = 0U; i < OUTPUT_COUNT; i++)
    {
        PWMOutput currentOutput =
            static_cast<PWMOutput>(i);

        const PWMOutputConfig& currentConfig =
            GetPWMOutputConfig(currentOutput);

        if (currentConfig.timer == timer)
        {
            ApplyDuty(i);
        }
    }

    // Apply PSC, ARR and compare changes promptly.
    __HAL_TIM_SET_COUNTER(timer, 0U);
    __HAL_TIM_GENERATE_EVENT(timer, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);

    return true;
}


// --------------------------------------------------
// Get actual PWM frequency from timer registers
// Uses HAL prescaler and auto-reload register macros
// --------------------------------------------------
uint32_t PWMOutputManager::GetFrequency(PWMOutput output)
{
    if (!IsValidOutput(output))
    {
        return 0U;
    }

    const PWMOutputConfig& config = GetPWMOutputConfig(output);

    uint32_t timerClock = GetTimerClock(config.timer);

    if (timerClock == 0U)
    {
        return 0U;
    }

    // Read actual programmed register values.
    uint32_t prescaler =
        __HAL_TIM_GET_PRESCALER(config.timer);

    uint32_t arr =
        __HAL_TIM_GET_AUTORELOAD(config.timer);

    uint64_t divisor =
        ((uint64_t)prescaler + 1ULL) *
        ((uint64_t)arr + 1ULL);

    if (divisor == 0ULL)
    {
        return 0U;
    }

    return (uint32_t)((uint64_t)timerClock / divisor);
}


// --------------------------------------------------
// Get last duty value requested through this manager
// --------------------------------------------------
float PWMOutputManager::GetDuty(PWMOutput output)
{
    if (!IsValidOutput(output))
    {
        return 0.0f;
    }

    return dutyPercent[ToIndex(output)];
}


// --------------------------------------------------
// Check manager's running state
// --------------------------------------------------
bool PWMOutputManager::IsRunning(PWMOutput output)
{
    if (!IsValidOutput(output))
    {
        return false;
    }

    return running[ToIndex(output)];
}
