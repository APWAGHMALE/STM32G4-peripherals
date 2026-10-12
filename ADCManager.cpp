
#include "ADCManager.hpp"

uint16_t ADCManager::rawValues[8] = {0};
float ADCManager::voltageValues[8] = {0.0f};

float ADCManager::referenceVoltage = 3.3f;
bool ADCManager::initialized = false;


// ==================================================
// INPUT VALIDATION
// ==================================================

bool ADCManager::IsValidInput(ADCInput input)
{
    uint8_t index = static_cast<uint8_t>(input);
    return index < 8U;
}


// ==================================================
// INITIALIZATION
// ==================================================

bool ADCManager::Init()
{
    initialized = false;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        rawValues[i] = 0U;
        voltageValues[i] = 0.0f;

        if (adcMap[i].adc == nullptr)
            return false;
    }

    // Calibrate ADC1.
    if (HAL_ADCEx_Calibration_Start(
            &hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }

    // Calibrate ADC2.
    if (HAL_ADCEx_Calibration_Start(
            &hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }

    initialized = true;
    return true;
}


// ==================================================
// CONFIGURE ONE ADC INPUT
// ==================================================

bool ADCManager::ConfigureInput(ADCInput input)
{
    if (!IsValidInput(input))
        return false;

    const ADCConfig& config = GetADCConfig(input);

    if (config.adc == nullptr)
        return false;

    ADC_ChannelConfTypeDef channelConfig = {};

    channelConfig.Channel = config.channel;
    channelConfig.Rank = ADC_REGULAR_RANK_1;
    channelConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    channelConfig.SingleDiff = ADC_SINGLE_ENDED;
    channelConfig.OffsetNumber = ADC_OFFSET_NONE;
    channelConfig.Offset = 0U;

    return HAL_ADC_ConfigChannel(
        config.adc,
        &channelConfig
    ) == HAL_OK;
}


// ==================================================
// READ RAW ADC VALUE
// ==================================================

bool ADCManager::ReadRaw(
    ADCInput input,
    uint16_t& value)
{
    if (!initialized || !IsValidInput(input))
        return false;

    const ADCConfig& config = GetADCConfig(input);

    if (config.adc == nullptr)
        return false;

    if (!ConfigureInput(input))
        return false;

    if (HAL_ADC_Start(config.adc) != HAL_OK)
        return false;

    if (HAL_ADC_PollForConversion(
            config.adc, 10U) != HAL_OK)
    {
        HAL_ADC_Stop(config.adc);
        return false;
    }

    uint32_t adcValue = HAL_ADC_GetValue(config.adc);

    if (HAL_ADC_Stop(config.adc) != HAL_OK)
        return false;

    value = (uint16_t)adcValue;

    uint8_t index = static_cast<uint8_t>(input);
    rawValues[index] = value;

    voltageValues[index] =
        ((float)value / 4095.0f) * referenceVoltage;

    return true;
}


// ==================================================
// READ VOLTAGE
// ==================================================

bool ADCManager::ReadVoltage(
    ADCInput input,
    float& voltage)
{
    uint16_t raw = 0U;

    if (!ReadRaw(input, raw))
        return false;

    voltage =
        ((float)raw / 4095.0f) * referenceVoltage;

    return true;
}


// ==================================================
// READ ALL RAW ADC VALUES
// ==================================================

bool ADCManager::ReadAll(uint16_t values[8])
{
    if (values == nullptr)
        return false;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (!ReadRaw(
                static_cast<ADCInput>(i),
                values[i]))
        {
            return false;
        }
    }

    return true;
}


// ==================================================
// READ ALL VOLTAGES
// ==================================================

bool ADCManager::ReadAllVoltages(float voltages[8])
{
    if (voltages == nullptr)
        return false;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (!ReadVoltage(
                static_cast<ADCInput>(i),
                voltages[i]))
        {
            return false;
        }
    }

    return true;
}


// ==================================================
// GET CACHED RAW VALUE
// ==================================================

uint16_t ADCManager::GetLastRaw(ADCInput input)
{
    if (!IsValidInput(input))
        return 0U;

    return rawValues[static_cast<uint8_t>(input)];
}


// ==================================================
// GET CACHED VOLTAGE
// ==================================================

float ADCManager::GetLastVoltage(ADCInput input)
{
    if (!IsValidInput(input))
        return 0.0f;

    return voltageValues[static_cast<uint8_t>(input)];
}


// ==================================================
// SET ADC REFERENCE VOLTAGE
// ==================================================

bool ADCManager::SetReferenceVoltage(float voltage)
{
    if (voltage <= 0.0f || voltage > 5.0f)
        return false;

    referenceVoltage = voltage;
    return true;
}
