
#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "main.h"

/*
 * STM32G491RE HIL CONTROLLER - MASTER PIN MAP
 *
 * ADC INPUTS : 8
 * PWM OUTPUTS: 12
 * PWM INPUTS : 8
 * FDCAN      : 2
 * USART      : 1
 *
 * Change pin assignments only in this file and PinMap.cpp.
 */

/* ADC INPUTS */

enum class ADCInput : uint8_t
{
    ADC_IN1 = 0,    // PA0 - ADC1_IN1
    ADC_IN2,        // PA4 - ADC2_IN17
    ADC_IN3,        // PA5 - ADC2_IN13
    ADC_IN4,        // PA6 - ADC2_IN3
    ADC_IN5,        // PA7 - ADC2_IN4
    ADC_IN6,        // PC0 - ADC12_IN6
    ADC_IN7,        // PC1 - ADC12_IN7
    ADC_IN8         // PC2 - ADC12_IN8
};

struct ADCConfig
{
    GPIO_TypeDef* port;
    uint16_t pin;
    ADC_HandleTypeDef* adc;
    uint32_t channel;
};

/* PWM OUTPUTS */

enum class PWMOutput : uint8_t
{
    PWM_OUT1 = 0,   // PA8  - TIM1_CH1
    PWM_OUT2,       // PA9  - TIM1_CH2
    PWM_OUT3,       // PA10 - TIM1_CH3
    PWM_OUT4,       // PC3  - TIM1_CH4

    PWM_OUT5,       // PB6  - TIM4_CH1
    PWM_OUT6,       // PB7  - TIM4_CH2
    PWM_OUT7,       // PB8  - TIM4_CH3
    PWM_OUT8,       // PB9  - TIM4_CH4

    PWM_OUT9,       // PA15 - TIM2_CH1
    PWM_OUT10,      // PA1  - TIM2_CH2
    PWM_OUT11,      // PA2  - TIM2_CH3
    PWM_OUT12       // PA3  - TIM2_CH4
};

struct PWMOutputConfig
{
    GPIO_TypeDef* port;
    uint16_t pin;
    TIM_HandleTypeDef* timer;
    uint32_t channel;
};

/* PWM INPUTS */

enum class PWMInput : uint8_t
{
    PWM_IN1 = 0,    // Reassign: no TIM2 channels available
    PWM_IN2,
    PWM_IN3,        // PB4 - TIM3_CH1
    PWM_IN4,        // PB5 - TIM3_CH2
    PWM_IN5,        // PB0 - TIM3_CH3
    PWM_IN6,        // PB1 - TIM3_CH4
    PWM_IN7,        // PC6 - TIM8_CH1
    PWM_IN8         // PC7 - TIM8_CH2
};

struct PWMInputConfig
{
    GPIO_TypeDef* port;
    uint16_t pin;
    TIM_HandleTypeDef* timer;
    uint32_t channel;
};

/* FDCAN */

enum class CANChannel : uint8_t
{
    CAN1 = 0,
    CAN2
};

struct CANConfig
{
    GPIO_TypeDef* rxPort;
    uint16_t rxPin;
    GPIO_TypeDef* txPort;
    uint16_t txPin;
    FDCAN_HandleTypeDef* fdcan;
};

/* USART */

struct USARTConfig
{
    GPIO_TypeDef* txPort;
    uint16_t txPin;
    GPIO_TypeDef* rxPort;
    uint16_t rxPin;
    UART_HandleTypeDef* uart;
};

/* EXTERNAL MAPS */

extern ADCConfig adcMap[8];
extern PWMOutputConfig pwmOutputMap[12];
extern PWMInputConfig pwmInputMap[8];
extern CANConfig canMap[2];
extern USARTConfig usart1Map;

/* ACCESS FUNCTIONS */

const ADCConfig& GetADCConfig(ADCInput input);
const PWMOutputConfig& GetPWMOutputConfig(PWMOutput output);
const PWMInputConfig& GetPWMInputConfig(PWMInput input);
const CANConfig& GetCANConfig(CANChannel channel);
const USARTConfig& GetUSART1Config();

#endif

