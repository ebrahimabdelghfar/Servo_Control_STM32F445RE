#ifndef __STM32F4XX_DEBUG_H
#define __STM32F4XX_DEBUG_H

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  format: String to be sent to the USART.
 * @retval None
 */
void printf_uart(const char *format, ...);

#endif /* __STM32F4XX_DEBUG_H */