
#ifndef ADC_MANAGER_HPP
#define ADC_MANAGER_HPP

#include "main.h"
#include "PinMap.h"
#include <stdint.h>

class ADCManager
{
public:
    static bool Init();

    // Read one input; input uses ADCInput from PinMap.h.
    static bool ReadRaw(ADCInput input, uint16_t& value);
    static bool ReadVoltage(ADCInput input, float& voltage);

    // Read all eight inputs in PinMap array order.
    static bool ReadAll(uint16_t values[8]);
    static bool ReadAllVoltages(float voltages[8]);

    // Cached values from the most recent successful read.
    static uint16_t GetLastRaw(ADCInput input);
    static float GetLastVoltage(ADCInput input);

    // Set the ADC reference voltage used for conversion.
    static bool SetReferenceVoltage(float voltage);

private:
    static bool IsValidInput(ADCInput input);
    static bool ConfigureInput(ADCInput input);

    static uint16_t rawValues[8];
    static float voltageValues[8];
    static float referenceVoltage;
    static bool initialized;
};

#endif
