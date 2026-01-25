#ifndef __STM32F4XX_BRUSHLESS_H
#define __STM32F4XX_BRUSHLESS_H
#include <stdint.h>
/**
 * @file STM32F4xx_Brushless.h
 * @brief Brushless Motor Driver Interface for STM32F4xx
 */

/**
 * @brief Initialize the brushless motor driver
 * @param None
 * @return None
 */
void Brushless_Init(void);

/**
 * @brief Set the speed of the brushless motor
 * @param speed Desired speed (-100.0 to 100.0)
 * @return None
 */
void setBrushlessSpeed(float speed);

/**
 * @brief Set the ESC pulse width directly
 * @param pulse_us Pulse width in microseconds
 * @return None
 */
void SetESCPulseWidth(float pulse_us);

/**
 * @brief Run a test sequence on the brushless motor
 * @param None
 * @return None
 */
void Run_Test_Sequence(void);

#endif /* __STM32F4XX_BRUSHLESS_H */