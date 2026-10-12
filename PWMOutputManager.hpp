#ifndef PWM_OUTPUT_MANAGER_HPP
#define PWM_OUTPUT_MANAGER_HPP

#include "PinMap.h"
#include <stdint.h>

class PWMOutputManager
{
public:
    static void Init();

    static bool Start(PWMOutput output);
    static bool Stop(PWMOutput output);

    static void StartAll();
    static void StopAll();

    static bool SetFrequency(PWMOutput output, uint32_t frequencyHz);
    static uint32_t GetFrequency(PWMOutput output);

    static bool SetDuty(PWMOutput output, float dutyPercent);
    static float GetDuty(PWMOutput output);

    static bool IsRunning(PWMOutput output);

private:
    static constexpr uint8_t OUTPUT_COUNT = 12U;

    static bool running[OUTPUT_COUNT];
    static float dutyPercent[OUTPUT_COUNT];

    static uint8_t ToIndex(PWMOutput output);
    static bool IsValidOutput(PWMOutput output);

    static uint32_t GetTimerClock(TIM_HandleTypeDef* timer);
    static bool ApplyDuty(uint8_t index);
};

#endif
