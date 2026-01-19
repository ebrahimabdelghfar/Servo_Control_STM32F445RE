#include <math.h> // For fmax, fmin
#include <stdint.h>
#include "stm32_f466xx_servo.h"
#include "servo_config.h"
#include "adc.h"
#include "tim.h"
#include "STM32_F4xx_PID.h"
#include "STM32F4xx_Servo_Filter.h"

ServoPID myServoPID;
Median3_t filterStage1;
KalmanState_t filterStage2;
uint16_t adc_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_avg_ready = 0;
volatile uint16_t adc_average = 0;

float last_angle = 0.0f; // To store the last known angle
float integralTerm = 0.0f;
float prevError    = 0.0f;
uint32_t lastUpdate = 0;

void Servo_Init(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  HAL_Delay(100); // Wait for ADC to stabilize
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  setServoPulseWidth(0.0f); // 0.0f effort = Center position
  PID_Init(&myServoPID, SERVO_KP, SERVO_KI, SERVO_KD, INTEGRAL_WINDUP_GUARD, OUTPUT_LIMIT);
  Median3_Init(&filterStage1);
  Kalman_Init(&filterStage2, 0.02f, SERVO_FILTER_Q_ANGLE, SERVO_FILTER_Q_VEL, SERVO_FILTER_R_MEAS);
}

void setServoPulseWidth(float effort)
{
  uint16_t pulse_width = (uint16_t)(PWM_CENTER + effort);
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
    // Map ADC average to Voltage
    float voltage = ((float)adc_average / ADC_RESOLUTION) * REF_NUCLEO_VOLTAGE;
    // Map Voltage to Angle
    float angle = (voltage / SERVO_MAX_VOLTAGE) * SERVO_MAX_ANGLE;
    float despiked_angle = Median3_Apply(&filterStage1, angle);
    float filtered_angle = Kalman_Update(&filterStage2, despiked_angle);
    last_angle = filtered_angle; // Update the last known angle
    return filtered_angle;
  }
  else
  {
    return last_angle; // Return the last known angle if no new data is available
  }
}

void setServoAngle(float angle)
{
  // Clamp angle to valid range
  if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

  // Read current angle
  float current_angle = servoReadAngle();
  // Compute PID output
  float pid_output = PID_Compute(&myServoPID, angle, current_angle);

  // Set servo pulse width based on PID output
  setServoPulseWidth(pid_output);
  
}

float readWrappedAngle(float angle)
{
  // Normalize angle to [0, 180]
  angle = fmodf(angle, 180.0f);
  if (angle < 0.0f) angle += 180.0f;
  return angle;
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