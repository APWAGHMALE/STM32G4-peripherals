
#ifndef PWM_OUTPUT_MANAGER_HPP
#define PWM_OUTPUT_MANAGER_HPP

#include "main.h"
#include "PinMap.h"
#include <stdint.h>

class PWMOutputManager
{
public:
    static bool Init();

    // Channel numbers: 1 to 12
    static bool Start(uint8_t channel);
    static bool Stop(uint8_t channel);

    static bool StartAll();
    static bool StopAll();

    // Frequency in Hz; shared by all channels on the same timer.
    static bool SetFrequency(uint8_t channel, uint32_t frequencyHz);
    static uint32_t GetFrequency(uint8_t channel);

    // Duty cycle in percent: 0.0 to 100.0
    static bool SetDuty(uint8_t channel, float duty);
    static float GetDuty(uint8_t channel);

    static bool IsRunning(uint8_t channel);

private:
    static bool IsValidChannel(uint8_t channel);
    static bool IsTimerRunning(TIM_HandleTypeDef* timer);

    static uint32_t GetTimerClock(TIM_HandleTypeDef* timer);

    static uint32_t DutyToCompare(
        const PWMOutputConfig& config,
        float duty
    );

    static bool running[12];
    static float dutyCycle[12];
};

#endif
