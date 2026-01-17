#include "STM32F4xx_Debug.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include "stdint.h"

static size_t append_char(char *dst, size_t dst_size, size_t idx, char c)
{
    if (idx + 1 < dst_size)
    {
        dst[idx++] = c;
        dst[idx] = '\0';
    }
    return idx;
}

static size_t append_str(char *dst, size_t dst_size, size_t idx, const char *src)
{
    if (src == NULL)
    {
        src = "(null)";
    }
    while (*src && idx + 1 < dst_size)
    {
        dst[idx++] = *src++;
    }
    if (idx < dst_size)
    {
        dst[idx] = '\0';
    }
    return idx;
}

static uint32_t pow10_u(uint32_t p)
{
    uint32_t result = 1U;
    while (p-- > 0U)
    {
        result *= 10U;
    }
    return result;
}

static size_t append_int_format(char *dst, size_t dst_size, size_t idx, const char *fmt, ...) 
{
    char temp[32];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);
    if (len > 0)
    {
        return append_str(dst, dst_size, idx, temp);
    }
    return idx;
}

static size_t append_float(char *dst, size_t dst_size, size_t idx, double value, int precision)
{
    if (precision < 0)
    {
        precision = 2;
    }
    if (precision > 6)
    {
        precision = 6;
    }

    if (value < 0.0)
    {
        idx = append_char(dst, dst_size, idx, '-');
        value = -value;
    }

    uint32_t scale = pow10_u((uint32_t)precision);
    uint32_t int_part = (uint32_t)value;
    double frac = value - (double)int_part;
    uint32_t frac_part = (uint32_t)((frac * (double)scale) + 0.5);

    if (frac_part >= scale)
    {
        int_part += 1U;
        frac_part = 0U;
    }

    idx = append_int_format(dst, dst_size, idx, "%lu", (unsigned long)int_part);
    if (precision > 0)
    {
        idx = append_char(dst, dst_size, idx, '.');
        idx = append_int_format(dst, dst_size, idx, "%0*lu", precision, (unsigned long)frac_part);
    }
    return idx;
}

volatile uint8_t uart_tx_busy = 0; // Flag to track DMA status
void printf_uart(const char *format, ...)
{
    static char msg_buffer[128]; // Buffer must persist for DMA transfer
    size_t idx = 0;
    va_list args;
    va_start(args, format);

    while (*format && idx + 1 < sizeof(msg_buffer))
    {
        if (*format != '%')
        {
            idx = append_char(msg_buffer, sizeof(msg_buffer), idx, *format++);
            continue;
        }

        format++;
        if (*format == '%')
        {
            idx = append_char(msg_buffer, sizeof(msg_buffer), idx, '%');
            format++;
            continue;
        }

        int precision = -1;
        if (*format == '.')
        {
            precision = 0;
            format++;
            while (isdigit((unsigned char)*format))
            {
                precision = (precision * 10) + (*format - '0');
                format++;
            }
        }

        int long_mod = 0;
        if (*format == 'l')
        {
            long_mod = 1;
            format++;
        }

        switch (*format)
        {
            case 'd':
            case 'i':
                if (long_mod)
                {
                    long v = va_arg(args, long);
                    idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%ld", v);
                }
                else
                {
                    int v = va_arg(args, int);
                    idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%d", v);
                }
                break;
            case 'u':
                if (long_mod)
                {
                    unsigned long v = va_arg(args, unsigned long);
                    idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%lu", v);
                }
                else
                {
                    unsigned int v = va_arg(args, unsigned int);
                    idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%u", v);
                }
                break;
            case 'x':
            case 'X':
            {
                unsigned int v = va_arg(args, unsigned int);
                idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, (*format == 'X') ? "%X" : "%x", v);
                break;
            }
            case 'c':
            {
                int c = va_arg(args, int);
                idx = append_char(msg_buffer, sizeof(msg_buffer), idx, (char)c);
                break;
            }
            case 's':
            {
                const char *s = va_arg(args, const char *);
                idx = append_str(msg_buffer, sizeof(msg_buffer), idx, s);
                break;
            }
            case 'f':
            {
                double v = va_arg(args, double);
                idx = append_float(msg_buffer, sizeof(msg_buffer), idx, v, precision);
                break;
            }
            default:
                idx = append_char(msg_buffer, sizeof(msg_buffer), idx, '%');
                if (*format)
                {
                    idx = append_char(msg_buffer, sizeof(msg_buffer), idx, *format);
                }
                break;
        }
        if (*format)
        {
            format++;
        }
    }

    va_end(args);

    if (idx > 0)
    {
        while (uart_tx_busy)
            ;

        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg_buffer, idx);
        if (status == HAL_OK)
        {
            uart_tx_busy = 1;
        }
        else
        {
            uart_tx_busy = 0;
            HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, idx, 100);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_busy = 0; // Clear the flag, ready for next message
    }
}