#include "STM32F4xx_Debug.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "stdint.h"

#define DEBUG_UART_RX_BUFFER_SIZE 128U

static volatile uint8_t debug_rx_byte = 0U;
static volatile uint8_t debug_rx_buffer[DEBUG_UART_RX_BUFFER_SIZE];
static volatile uint16_t debug_rx_head = 0U;
static volatile uint16_t debug_rx_tail = 0U;
static volatile uint8_t debug_rx_overflow = 0U;
static volatile uint8_t debug_rx_started = 0U;
static volatile uint8_t debug_uart_echo_enabled = 1U;
static volatile uint8_t debug_uart_echo_busy = 0U;
static volatile uint8_t debug_uart_echo_byte = 0U;

static uint16_t debug_uart_advance(uint16_t idx)
{
    idx++;
    if (idx >= DEBUG_UART_RX_BUFFER_SIZE)
    {
        idx = 0U;
    }
    return idx;
}

static void debug_uart_rx_push(uint8_t byte)
{
    uint16_t next = debug_uart_advance(debug_rx_head);
    if (next == debug_rx_tail)
    {
        debug_rx_overflow = 1U;
        return;
    }
    debug_rx_buffer[debug_rx_head] = byte;
    debug_rx_head = next;
}

static void debug_uart_rx_ensure_started(void)
{
    if (debug_rx_started == 0U)
    {
        debug_rx_started = 1U;
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&debug_rx_byte, 1U);
    }
}

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

static size_t append_padded_uint(char *dst, size_t dst_size, size_t idx, unsigned long value, int width, char pad, int is_upper, int is_hex)
{
    char temp[32];
    int len = 0;

    if (is_hex)
    {
        len = snprintf(temp, sizeof(temp), is_upper ? "%lX" : "%lx", value);
    }
    else
    {
        len = snprintf(temp, sizeof(temp), "%lu", value);
    }

    if (len <= 0)
    {
        return idx;
    }

    int pad_count = (width > len) ? (width - len) : 0;
    while (pad_count-- > 0 && idx + 1 < dst_size)
    {
        dst[idx++] = pad;
    }

    return append_str(dst, dst_size, idx, temp);
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
        int width = 0;
        char pad_char = ' ';

        if (*format == '0')
        {
            pad_char = '0';
            format++;
        }

        while (isdigit((unsigned char)*format))
        {
            width = (width * 10) + (*format - '0');
            format++;
        }

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
                    if (width > 0)
                    {
                        idx = append_padded_uint(msg_buffer, sizeof(msg_buffer), idx, v, width, pad_char, 0, 0);
                    }
                    else
                    {
                        idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%lu", v);
                    }
                }
                else
                {
                    unsigned int v = va_arg(args, unsigned int);
                    if (width > 0)
                    {
                        idx = append_padded_uint(msg_buffer, sizeof(msg_buffer), idx, (unsigned long)v, width, pad_char, 0, 0);
                    }
                    else
                    {
                        idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, "%u", v);
                    }
                }
                break;
            case 'x':
            case 'X':
            {
                unsigned int v = va_arg(args, unsigned int);
                if (width > 0)
                {
                    idx = append_padded_uint(msg_buffer, sizeof(msg_buffer), idx, (unsigned long)v, width, pad_char, (*format == 'X'), 1);
                }
                else
                {
                    idx = append_int_format(msg_buffer, sizeof(msg_buffer), idx, (*format == 'X') ? "%X" : "%x", v);
                }
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
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, idx, 100);
    }
}

void debug_uart_rx_init(void)
{
    debug_uart_rx_clear();
    debug_rx_started = 1U;
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&debug_rx_byte, 1U);
}

size_t debug_uart_available(void)
{
    uint16_t head = debug_rx_head;
    uint16_t tail = debug_rx_tail;

    if (head >= tail)
    {
        return (size_t)(head - tail);
    }
    return (size_t)(DEBUG_UART_RX_BUFFER_SIZE - tail + head);
}

int debug_uart_getchar(void)
{
    debug_uart_rx_ensure_started();
    if (debug_rx_head == debug_rx_tail)
    {
        return -1;
    }

    uint8_t byte = debug_rx_buffer[debug_rx_tail];
    debug_rx_tail = debug_uart_advance(debug_rx_tail);
    return (int)byte;
}

int debug_uart_readline(char *out, size_t out_size)
{
    debug_uart_rx_ensure_started();
    if ((out == NULL) || (out_size == 0U))
    {
        return 0;
    }

    uint16_t head = debug_rx_head;
    uint16_t tail = debug_rx_tail;
    if (head == tail)
    {
        out[0] = '\0';
        return 0;
    }

    uint16_t idx = tail;
    uint8_t found = 0U;
    while (idx != head)
    {
        uint8_t c = debug_rx_buffer[idx];
        if ((c == '\n') || (c == '\r'))
        {
            found = 1U;
            break;
        }
        idx = debug_uart_advance(idx);
    }

    if (found == 0U)
    {
        out[0] = '\0';
        return 0;
    }

    size_t out_idx = 0U;
    while (debug_rx_tail != head)
    {
        uint8_t c = debug_rx_buffer[debug_rx_tail];
        debug_rx_tail = debug_uart_advance(debug_rx_tail);

        if ((c == '\n') || (c == '\r'))
        {
            if ((c == '\r') && (debug_rx_tail != head))
            {
                uint8_t next = debug_rx_buffer[debug_rx_tail];
                if (next == '\n')
                {
                    debug_rx_tail = debug_uart_advance(debug_rx_tail);
                }
            }
            break;
        }

        if ((out_idx + 1U) < out_size)
        {
            out[out_idx++] = (char)c;
        }
    }

    out[out_idx] = '\0';
    return (int)out_idx;
}

uint8_t debug_uart_rx_overflowed(void)
{
    return debug_rx_overflow;
}

void debug_uart_rx_clear(void)
{
    debug_rx_head = 0U;
    debug_rx_tail = 0U;
    debug_rx_overflow = 0U;
    debug_rx_started = 0U;
    memset((void *)debug_rx_buffer, 0, sizeof(debug_rx_buffer));
}

int debug_uart_scanf(const char *format, ...)
{
    char line[128];
    int len = 0;

    while (len == 0)
    {
        len = debug_uart_readline(line, sizeof(line));
        if (len == 0)
        {
            HAL_Delay(1);
        }
    }

    va_list args;
    va_start(args, format);
    int parsed = vsscanf(line, format, args);
    va_end(args);

    return parsed;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_busy = 0; // Clear the flag, ready for next message
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        debug_uart_rx_push(debug_rx_byte);
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&debug_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&debug_rx_byte, 1U);
    }
}