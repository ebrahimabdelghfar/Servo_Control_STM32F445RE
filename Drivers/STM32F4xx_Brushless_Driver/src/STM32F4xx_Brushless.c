#include "STM32F4xx_Brushless.h"
#include "brushless_config.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

void Brushless_Init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  SetESCPulseWidth(NEUTRAL); // 0.0f effort = 0% duty cycle
}

void SetESCPulseWidth(float pulse_us)
{
  // Safety Clamping
  if (pulse_us < MIN_THROTTLE) pulse_us = MIN_THROTTLE;
  if (pulse_us > MAX_THROTTLE) pulse_us = MAX_THROTTLE;
  // Update Timer Compare Register (Duty Cycle)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)pulse_us);
}

void setBrushlessSpeed(float speed)
{
  static float currentThrottle = NEUTRAL;
  float targetThrottle;

  // Apply expo curve to increase sensitivity around small changes
  float x = speed / 100.0f;
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  float expo = BRUSHLESS_EXPO;
  if (expo < 0.0f) expo = 0.0f;
  if (expo > 1.0f) expo = 1.0f;
  float x3 = x * x * x;
  float shaped = (1.0f - expo) * x + expo * x3;

  // Map speed (-100.0 to 100.0) to pulse width
  if (shaped > 0.0f)
  {
    targetThrottle = NEUTRAL + ((MAX_THROTTLE - NEUTRAL) * shaped);
  }
  else if (shaped < 0.0f)
  {
    targetThrottle = NEUTRAL + ((NEUTRAL - MIN_THROTTLE) * shaped);
  }
  else
  {
    targetThrottle = NEUTRAL;
  }

  // Ramp towards target throttle
  if (currentThrottle < targetThrottle)
  {
    currentThrottle += RAMP_DELAY;
    if (currentThrottle > targetThrottle)
      currentThrottle = targetThrottle;
  }
  else if (currentThrottle > targetThrottle)
  {
    currentThrottle -= RAMP_DELAY;
    if (currentThrottle < targetThrottle)
      currentThrottle = targetThrottle;
  }

  SetESCPulseWidth(currentThrottle);
}