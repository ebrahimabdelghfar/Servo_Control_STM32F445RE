#include <math.h> // For fmax, fmin
#include <stdint.h>
#include "stm32_f466xx_servo.h"
#include "servo_config.h"
#include "adc.h"
#include "tim.h"
#include "STM32F4xx_Debug.h"
uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_avg_ready = 0;
volatile uint16_t adc_average = 0;

float last_angle = 0.0f; // To store the last known angle
float integralTerm = 0.0f;
float prevError    = 0.0f;
uint32_t lastUpdate = 0;

void Servo_Init(void)
{
  printf_uart("Initializing ADC...\r\n");
  if (hadc1.DMA_Handle == NULL)
  {
    printf_uart("ADC DMA handle is NULL\r\n");
    return;
  }

  HAL_ADC_Stop_DMA(&hadc1);
  HAL_StatusTypeDef adc_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  if (adc_status != HAL_OK)
  {
    uint32_t adc_error = HAL_ADC_GetError(&hadc1);
    printf_uart("ADC DMA start failed: status=%lu error=0x%08lx state=0x%08lx\r\n",
                (uint32_t)adc_status, (uint32_t)adc_error, (uint32_t)hadc1.State);
    return;
  }
  printf_uart("Initializing PWM...\r\n");
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  setServoPulseWidth(1500); // 1500us = Stop for continuous servo
}

float map_voltage_to_angle(float voltage, float min_angle, float max_angle)
{
  return (voltage - 0.0f) * (max_angle - min_angle) / (SERVO_MAX_VOLTAGE - 0.0f) + min_angle;
}

void setServoPulseWidth(uint16_t pulse_width)
{
  // Safety Clamping
  if (pulse_width < MIN_PWM_PULSE_WIDTH) pulse_width = MIN_PWM_PULSE_WIDTH;
  if (pulse_width > MAX_PWM_PULSE_WIDTH) pulse_width = MAX_PWM_PULSE_WIDTH;
  // Update Timer Compare Register (Duty Cycle)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
}

float servoReadAngle(void)
{

  if (adc_avg_ready != 0)
  {
    adc_avg_ready = 0;
    float adc_value = (float)adc_average; // Use the averaged ADC value
    float voltage = (float)((adc_value) * (REF_NUCLEO_VOLTAGE / ADC_RESOLUTION));
    float angle = map_voltage_to_angle(voltage, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    last_angle = angle; // Update the last known angle
    return angle;
  }
  else
  {
    return last_angle; // Return the last known angle if no new data is available
  }
}

void setServoAngle(float angle)
{
  uint32_t now = HAL_GetTick(); // Returns milliseconds
  float dt = (now - lastUpdate) / 1000.0f;
  if (dt <= 0.0f) return 0.0f; // Prevent division by zero
  float error = angle - servoReadAngle();
  integralTerm += error * dt;
  // Constrain Integral
  if (integralTerm > INTEGRAL_WINDUP_GUARD) integralTerm = INTEGRAL_WINDUP_GUARD;
  if (integralTerm < -INTEGRAL_WINDUP_GUARD) integralTerm = -INTEGRAL_WINDUP_GUARD;
  // Derivative
  float derivative = (error - prevError) / dt;
  // Calculate Output
  float output = (SERVO_KP * error) + (SERVO_KI * integralTerm) + (SERVO_KD * derivative);
  // Save history
  prevError = error;
  lastUpdate = now;
  // Clamp Output
  if (output > OUTPUT_LIMIT) output = OUTPUT_LIMIT;
  if (output < -OUTPUT_LIMIT) output = -OUTPUT_LIMIT;
  // Here we map +/- controlEffort to 1500us.
  // We multiply by ~11 (since 1 degree ~ 11us pulse width change) to keep PID gains similar
  int servoCommand_us = 1500 + (int)(output * 11.1f);
  setServoPulseWidth(servoCommand_us);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint32_t sum = 0;

    for (uint32_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
      sum += (adc_buffer[i] & 0x0FFFu);
    }

    adc_average = sum / ADC_BUFFER_SIZE;
    adc_avg_ready = 1;
  }
}