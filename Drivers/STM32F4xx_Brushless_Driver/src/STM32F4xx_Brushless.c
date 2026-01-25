#include "STM32F4xx_Brushless.h"
#include "brushless_config.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

void Brushless_Init(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  SetESCPulseWidth(NEUTRAL); // 0.0f effort = 0% duty cycle
}

void SetESCPulseWidth(float pulse_us)
{
  // Safety Clamping
  if (pulse_us < MIN_THROTTLE) pulse_us = MIN_THROTTLE;
  if (pulse_us > MAX_THROTTLE) pulse_us = MAX_THROTTLE;
  // Update Timer Compare Register (Duty Cycle)
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)pulse_us);
}

void setBrushlessSpeed(float speed)
{
  static float currentThrottle = NEUTRAL;
  float targetThrottle;

  // Map speed (-100.0 to 100.0) to pulse width
  if (speed > 0.0f)
  {
    targetThrottle = NEUTRAL + ((MAX_THROTTLE - NEUTRAL) * (speed / 100.0f));
  }
  else if (speed < 0.0f)
  {
    targetThrottle = NEUTRAL + ((NEUTRAL - MIN_THROTTLE) * (speed / 100.0f));
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