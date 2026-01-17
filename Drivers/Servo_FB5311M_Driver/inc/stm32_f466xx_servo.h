#ifndef __STM32_F466XX_SERVO_H
#define __STM32_F466XX_SERVO_H

#include <stdint.h>

/**
 * @file stm32_f466xx_servo.h
 * @brief Header file for STM32F466xx Servo Driver
*/

/**
 * @brief Initialize the servo motor
 * @return None
 */
void Servo_Init(void);

/**
 * @brief Set the servo angle
 * @param angle: Desired servo angle in degrees
 * @return None
 */
void setServoAngle(float angle);

/**
 * @brief Set the servo pulse width
 * @param pulse_width: Desired pulse width in microseconds
 * @return None
 */
void setServoPulseWidth(uint16_t pulse_width);

/**
 * @brief Read the current servo angle
 * @return Current servo angle in degrees
 */
float servoReadAngle(void);

/**
 * @brief Map a voltage value to a servo angle
 * @param voltage: Input voltage
 * @param min_angle: Minimum angle corresponding to minimum voltage
 * @param max_angle: Maximum angle corresponding to maximum voltage
 * @return Mapped servo angle in degrees
 */
float map_voltage_to_angle(float voltage,float min_angle,float max_angle);

#endif /* __STM32_F466XX_SERVO_H */