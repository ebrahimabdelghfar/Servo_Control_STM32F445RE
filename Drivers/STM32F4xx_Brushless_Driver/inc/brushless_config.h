#ifndef __STM32F4XX_BRUSHLESS_CONFIG_H
#define __STM32F4XX_BRUSHLESS_CONFIG_H

#define MAX_THROTTLE 2000.0f
#define NEUTRAL      1500.0f
#define MIN_THROTTLE 1000.0f
#define RAMP_DELAY   10.0f
#define BRUSHLESS_PWM_FREQUENCY 50.0f // Maximum control loop frequency in Hz
#define BRUSHLESS_EXPO 0.05f            // 0.0f = linear, 1.0f = strong expo

#endif /* __STM32F4XX_BRUSHLESS_CONFIG_H */