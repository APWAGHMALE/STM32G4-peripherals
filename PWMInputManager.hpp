#ifndef PWM_INPUT_MANAGER_HPP
#define PWM_INPUT_MANAGER_HPP

#include "PinMap.hpp"
#include <stdint.h>

class PWMInputManager
{
public:
    static void Init();

    static bool Start(PWMInput input);
    static bool Stop(PWMInput input);

    static void StartAll();
    static void StopAll();

    static uint32_t GetFrequency(PWMInput input);
    static float GetDuty(PWMInput input);

    static uint32_t GetPeriodUs(PWMInput input);
    static uint32_t GetPulseWidthUs(PWMInput input);

    static bool IsValid(PWMInput input);
    static bool IsRunning(PWMInput input);

    static void CaptureCallback(TIM_HandleTypeDef* htim);
    static void Update();

private:
    static constexpr uint8_t CHANNEL_COUNT = 8U;
    static constexpr uint32_t COUNTER_TARGET_HZ = 1000000U;
    static constexpr uint32_t SIGNAL_TIMEOUT_MS = 2000U;

    static bool running[CHANNEL_COUNT];
    static bool haveRising[CHANNEL_COUNT];
    static bool waitingForFalling[CHANNEL_COUNT];
    static bool valid[CHANNEL_COUNT];

    static uint32_t lastRising[CHANNEL_COUNT];
    static uint32_t periodTicks[CHANNEL_COUNT];
    static uint32_t highTicks[CHANNEL_COUNT];
    static uint32_t counterClockHz[CHANNEL_COUNT];
    static uint32_t lastCaptureTick[CHANNEL_COUNT];

    static uint8_t ToIndex(PWMInput input);
    static bool IsValidInput(PWMInput input);

    static uint32_t GetTimerClock(TIM_HandleTypeDef* timer);
    static bool ConfigureTimer(TIM_HandleTypeDef* timer);
    static uint32_t GetCounterClock(PWMInput input);

    static uint32_t CalculateDelta(
        uint32_t current,
        uint32_t previous,
        uint32_t counterPeriod);
};

#endif
