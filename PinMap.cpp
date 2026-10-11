#include "PinMap.h"

/* CubeMX peripheral handles */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern UART_HandleTypeDef huart1;

/* ADC MAP */

ADCConfig adcMap[8] =
{
    {GPIOA, GPIO_PIN_0, &hadc1, ADC_CHANNEL_1},
    {GPIOA, GPIO_PIN_4, &hadc2, ADC_CHANNEL_17},
    {GPIOA, GPIO_PIN_5, &hadc2, ADC_CHANNEL_13},
    {GPIOA, GPIO_PIN_6, &hadc2, ADC_CHANNEL_3},
    {GPIOA, GPIO_PIN_7, &hadc2, ADC_CHANNEL_4},
    {GPIOC, GPIO_PIN_0, &hadc1, ADC_CHANNEL_6},
    {GPIOC, GPIO_PIN_1, &hadc1, ADC_CHANNEL_7},
    {GPIOC, GPIO_PIN_2, &hadc1, ADC_CHANNEL_8}
};

/* PWM OUTPUT MAP: TIM1 + TIM4 + TIM2 */

PWMOutputConfig pwmOutputMap[12] =
{
    {GPIOA, GPIO_PIN_8,  &htim1, TIM_CHANNEL_1},
    {GPIOA, GPIO_PIN_9,  &htim1, TIM_CHANNEL_2},
    {GPIOA, GPIO_PIN_10, &htim1, TIM_CHANNEL_3},
    {GPIOC, GPIO_PIN_3,  &htim1, TIM_CHANNEL_4},

    {GPIOB, GPIO_PIN_6,  &htim4, TIM_CHANNEL_1},
    {GPIOB, GPIO_PIN_7,  &htim4, TIM_CHANNEL_2},
    {GPIOB, GPIO_PIN_8,  &htim4, TIM_CHANNEL_3},
    {GPIOB, GPIO_PIN_9,  &htim4, TIM_CHANNEL_4},

    {GPIOA, GPIO_PIN_15, &htim2, TIM_CHANNEL_1},
    {GPIOA, GPIO_PIN_1,  &htim2, TIM_CHANNEL_2},
    {GPIOA, GPIO_PIN_2,  &htim2, TIM_CHANNEL_3},
    {GPIOA, GPIO_PIN_3,  &htim2, TIM_CHANNEL_4}
};

/*
 * PWM INPUT MAP
 *
 * TIM2 is now dedicated to PWM output generation.
 * PWM_IN1 and PWM_IN2 must be assigned to alternate,
 * CubeMX-verified input-capture pins before use.
 */

PWMInputConfig pwmInputMap[8] =
{
    {GPIOB, GPIO_PIN_4, &htim3, TIM_CHANNEL_1},
    {GPIOB, GPIO_PIN_5, &htim3, TIM_CHANNEL_2},
    {GPIOB, GPIO_PIN_0, &htim3, TIM_CHANNEL_3},
    {GPIOB, GPIO_PIN_1, &htim3, TIM_CHANNEL_4},

    {GPIOC, GPIO_PIN_6, &htim8, TIM_CHANNEL_1},
    {GPIOC, GPIO_PIN_7, &htim8, TIM_CHANNEL_2},

    // Placeholders: assign valid TIM15 input-capture pins in CubeMX.
    {GPIOB, GPIO_PIN_14, &htim15, TIM_CHANNEL_1},
    {GPIOB, GPIO_PIN_15, &htim15, TIM_CHANNEL_2}
};

/* FDCAN MAP */

CANConfig canMap[2] =
{
    {GPIOA, GPIO_PIN_11, GPIOA, GPIO_PIN_12, &hfdcan1},
    {GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &hfdcan2}
};

/* USART MAP */

USARTConfig usart1Map =
{
    GPIOC, GPIO_PIN_4,
    GPIOC, GPIO_PIN_5,
    &huart1
};

/* ACCESS FUNCTIONS */

const ADCConfig& GetADCConfig(ADCInput input)
{
    return adcMap[static_cast<uint8_t>(input)];
}

const PWMOutputConfig& GetPWMOutputConfig(PWMOutput output)
{
    return pwmOutputMap[static_cast<uint8_t>(output)];
}

const PWMInputConfig& GetPWMInputConfig(PWMInput input)
{
    return pwmInputMap[static_cast<uint8_t>(input)];
}

const CANConfig& GetCANConfig(CANChannel channel)
{
    return canMap[static_cast<uint8_t>(channel)];
}

const USARTConfig& GetUSART1Config()
{
    return usart1Map;
}
