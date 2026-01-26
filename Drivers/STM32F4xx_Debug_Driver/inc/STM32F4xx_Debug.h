#ifndef __STM32F4XX_DEBUG_H
#define __STM32F4XX_DEBUG_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  format: String to be sent to the USART.
 * @retval None
 */
void printf_uart(const char *format, ...);

/**
 * @brief  Initialize UART RX input capture for debug console.
 *         Call after MX_USART2_UART_Init().
 * @retval None
 */
void debug_uart_rx_init(void);

/**
 * @brief  Return number of bytes currently buffered.
 * @retval Available byte count
 */
size_t debug_uart_available(void);

/**
 * @brief  Get one byte from the debug RX buffer (non-blocking).
 * @retval Byte value 0-255, or -1 if no data
 */
int debug_uart_getchar(void);

/**
 * @brief  Get one integer from a complete line (non-blocking).
 * @param  out_value: destination for parsed integer
 * @retval 1 on success, 0 if no complete line, -1 on parse/arg error
 */
int debug_uart_getint(int *out_value);

/**
 * @brief  Get one float from a complete line (non-blocking).
 * @param  out_value: destination for parsed float
 * @retval 1 on success, 0 if no complete line, -1 on parse/arg error
 */
int debug_uart_getfloat(float *out_value);

/**
 * @brief  Read one line (terminated by \r or \n) from RX buffer.
 * @param  out: destination buffer
 * @param  out_size: size of destination buffer
 * @retval Number of characters copied (0 if no complete line)
 */
int debug_uart_readline(char *out, size_t out_size);

/**
 * @brief  Check if RX buffer overflow occurred.
 * @retval 1 if overflow happened, 0 otherwise
 */
uint8_t debug_uart_rx_overflowed(void);

/**
 * @brief  Clear RX buffer and overflow flag.
 * @retval None
 */
void debug_uart_rx_clear(void);

/**
 * @brief  scanf-like helper that blocks until a line is received.
 * @param  format: scanf-style format string
 * @retval Number of successfully parsed items
 */
int debug_uart_scanf(const char *format, ...);

/**
 * @brief  Enable or disable RX echo (each received byte sent back).
 * @param  enable: 1 to enable, 0 to disable
 * @retval None
 */
void debug_uart_set_echo(uint8_t enable);

#endif /* __STM32F4XX_DEBUG_H */