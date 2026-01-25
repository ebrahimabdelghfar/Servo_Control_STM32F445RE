# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32F446xx (Cortex-M4F) embedded project implementing a closed-loop servo controller with ADC feedback, PWM output, and hybrid filtering. The project is generated from STM32CubeMX and built using `arm-none-eabi-gcc` via GNU Make.

## Build Commands

```bash
# Clean and build the project
make clean && make

# Clean only
make clean

# Build only
make
```

Build artifacts are written to `build/`:
- `build/test_multi_adc_pwm.elf` - ELF executable
- `build/test_multi_adc_pwm.hex` - Intel HEX format for flashing
- `build/test_multi_adc_pwm.bin` - Raw binary

## Flashing

```bash
# STM32CubeProgrammer CLI
STM32_Programmer_CLI -c port=SWD -w build/test_multi_adc_pwm.hex -v -rst

# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/test_multi_adc_pwm.elf verify reset exit"
```

## Architecture

### Core Application Flow

The application initializes HAL, clocks, and peripherals (GPIO, DMA, ADC, UART, TIM), then calls `Servo_Init()` which sets up:
- ADC1 with DMA circular mode for continuous analog feedback reading
- TIM2 CH1 for PWM servo control
- PID controller
- 3-tap median filter + 1D Kalman filter cascade

### Driver Layer Structure

The project uses a custom driver layer under `Drivers/STM32F4xx_*_Driver/`:

- **STM32F4xx_Debug_Driver**: Provides `printf_uart()` - a custom printf implementation retargeted to USART2 with DMA support. Does NOT use newlib's printf.

- **STM32F4xx_Servo_Driver**: Main servo control logic in `STM32F4xx_servo.c`. Implements:
  - `Servo_Init()` - Initializes ADC DMA, PWM, PID, and filters
  - `servoReadAngle()` - Reads ADC, averages buffer, maps voltage to angle, applies median then Kalman filter
  - `setServoAngle()` - Sets target angle via PID control loop
  - `setServoPulseWidth()` - Direct PWM pulse width control
  - ADC conversion complete callback `HAL_ADC_ConvCpltCallback()` - Computes running average from DMA buffer

- **STM32F4xx_PID_Driver**: PID controller implementation with anti-windup, output limits, and deadband compensation. Used by the servo driver.

- **STM32F4xx_Filter_Driver**: Hybrid filtering for circular angular data:
  - `Median3_t` - 3-tap median filter for spike removal
  - `KalmanState_t` - 1D Kalman filter with position/velocity state, auto-computes `dt` via `HAL_GetTick()`
  - Helper functions `constrainAngle_360()` and `angleDiff()` for circular angle math

- **STM32F4xx_Brushless_Driver**: (New, untracked) Brushless motor/ESC control using TIM3 CH2.

### Hardware Configuration

- **MCU**: STM32F446RETx
- **Clock**: 180 MHz via PLL from HSI
- **ADC1**: Channel 1 (PA1) for servo feedback voltage, continuous mode with DMA2_Stream0
- **TIM2 CH1**: PWM output for servo (PA0), 50 Hz (prescaler=89, period=19999)
- **TIM3 CH2**: PWM output for brushless motor (PA7)
- **USART2**: Debug output (PA2=TX, PA3=RX) with DMA1_Stream5/6

### Configuration Files

- `test_multi_adc_pwm.ioc` - STM32CubeMX configuration file (edit here to modify pin mappings, clocks, peripheral settings)
- `Drivers/STM32F4xx_Servo_Driver/inc/servo_config.h` - Tunable parameters (PID gains, filter noise values, PWM limits, ADC buffer size, voltage mapping)
- `Drivers/STM32F4xx_Brushless_Driver/inc/brushless_config.h` - Brushless motor parameters

### Important Code Patterns

- **USER CODE sections**: HAL-generated files (`main.c`, `adc.c`, `tim.c`, etc.) contain `/* USER CODE BEGIN */` and `/* USER CODE END */` markers. Only edit within these sections - regenerating from CubeMX will overwrite other code.

- **ADC DMA Buffer**: Averaging happens in the ADC conversion complete callback. The buffer size and averaging logic are in `servo_config.h`.

- **Angle Filtering Cascade**: Raw ADC → Voltage → Angle → Median3 → Kalman → final filtered angle. This handles sensor spikes and smooths noisy feedback.

- **Case-Sensitive Include Paths**: On Linux, include paths in the Makefile must match folder case exactly. Note `STM32F4XX_Filter_Driver` uses uppercase `XX`.