# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Commands

- **Build the firmware** (clean then compile):
  ```bash
  make clean && make
  ```
  The build produces `build/test_multi_adc_pwm.elf`, `.hex`, and `.bin`.

- **Flash the firmware** – two examples:
  ```bash
  # STM32CubeProgrammer CLI
  STM32_Programmer_CLI -c port=SWD -w build/test_multi_adc_pwm.hex -v -rst
  ```
  ```bash
  # OpenOCD
  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/test_multi_adc_pwm.elf verify reset exit"
  ```

- **Clean build artifacts**:
  ```bash
  make clean
  ```

There are no automated unit‑test suites; runtime behaviour is verified via UART output.

## High‑Level Architecture Overview

- **Core entry point** – `Core/Src/main.c` initializes the HAL, clocks, GPIO, ADC/DMA, UART, and TIM peripherals, then calls `Servo_Init()` and enters the main loop.

- **Servo driver** – `Drivers/STM32F4xx_Servo_Driver/src/stm32F4xx_servo.c` provides low‑level PWM configuration and `setServoAngle()` to command a fixed angle.

- **PID controller** – `Drivers/STM32F4xx_PID_Driver/src/STM32F4xx_PID.c` implements the PID algorithm that calculates the PWM duty based on the filtered angle error.

- **Filtering** – `Drivers/STM32F4xx_Filter_Driver/src/STM32F4xx_Servo_Filter.c` applies a 3‑tap median filter (`Median3_Apply`) followed by a 1‑D Kalman filter (`Kalman_Update`). The Kalman filter automatically computes the time delta via `HAL_GetTick()`.

- **ADC acquisition** – `Core/Src/adc.c` (with DMA) samples the analog voltage from the potentiometer, buffers it, and `adc.c` supplies the raw average voltage to the filter pipeline.

- **UART debugging** – `Core/Src/usart.c` retargets `printf` to USART2, allowing real‑time angle values and PID diagnostics to be printed.

- **Configuration** – `Drivers/STM32F4xx_Servo_Driver/inc/servo_config.h` holds PWM frequency, limits, PID gains, and filter parameters. Adjusting these values tunes the control loop without recompiling the driver code.

- **Build system** – The `Makefile` orchestrates compilation with the ARM GCC toolchain (`arm-none-eabi-gcc`). Sources are listed under `C_SOURCES`; include paths cover core, HAL, and driver directories. The target binary is built in the `build/` directory.

## Project Structure Highlights

- `Core/` – generic MCU startup code, system clock config, and peripheral drivers (GPIO, ADC, DMA, UART, TIM).
- `Drivers/STM32F4xx_HAL_Driver/` – vendor HAL sources.
- `Drivers/STM32F4xx_Servo_Driver/`, `PID_Driver/`, `Filter_Driver/`, `Brushless_Driver/` – modular drivers for PWM output, control algorithm, filtering, and brushless motor handling.
- `Makefile` – GNU‑make based build script; respects `DEBUG` flag and `OPT` settings.
- `test_multi_adc_pwm.ioc` – STM32CubeMX project file describing peripheral configuration (ADC1, TIM2 CH1, USART2, etc.).

These notes give Claude Code enough context to navigate the codebase, run builds, flash the device, and understand where to modify control parameters or add new functionality.