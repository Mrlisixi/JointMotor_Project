#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usbd_core.h"
#include "cdc_class.h"

extern usbd_core_type usb_core_dev;

/**
 * @brief 将浮点数转换为字符串
 * @param value 浮点数
 * @param buf 输出缓冲区
 * @param precision 小数位数
 * @return 转换后的字符串长度
 */
int float_to_string(float value, char *buf, int precision)
{
    int integer_part = (int)value;
    float fractional_part = value - integer_part;
    int len = 0;
    
    // 处理负号
    if (value < 0)
    {
        buf[len++] = '-';
        integer_part = -integer_part;
        fractional_part = -fractional_part;
    }
    
    // 转换整数部分
    char int_buf[10];
    int int_len = 0;
    if (integer_part == 0)
    {
        int_buf[int_len++] = '0';
    }
    else
    {
        while (integer_part > 0)
        {
            int_buf[int_len++] = (integer_part % 10) + '0';
            integer_part /= 10;
        }
        // 反转整数部分
        for (int i = 0; i < int_len / 2; i++)
        {
            char temp = int_buf[i];
            int_buf[i] = int_buf[int_len - 1 - i];
            int_buf[int_len - 1 - i] = temp;
        }
    }
    
    // 复制整数部分到输出缓冲区
    memcpy(buf + len, int_buf, int_len);
    len += int_len;
    
    // 处理小数部分
    if (precision > 0)
    {
        buf[len++] = '.';
        for (int i = 0; i < precision; i++)
        {
            fractional_part *= 10;
            int digit = (int)fractional_part;
            buf[len++] = digit + '0';
            fractional_part -= digit;
        }
    }
    
    buf[len] = '\0';
    return len;
}

/**
 * @brief 发送字符串到USB
 * @param str 字符串
 * @return 发送的字符数
 */
int usb_send_string(const char *str)
{
    int len = strlen(str);
    error_status status;
    uint16_t i = 0;
    
    while (i < len)
    {
        do
        {
            status = usb_vcp_send_data(&usb_core_dev, (uint8_t *)&str[i], 1);
        } while (status == ERROR);
        i++;
    }
    
    return len;
}

/**
 * @brief 发送浮点数到USB
 * @param value 浮点数
 * @param precision 小数位数
 * @return 发送的字符数
 */
int usb_send_float(float value, int precision)
{
    char buf[20];
    float_to_string(value, buf, precision);
    return usb_send_string(buf);
}

/**
 * @brief 重定向printf到USB
 */
int _write(int file, char *ptr, int len)
{
    // 只处理标准输出和标准错误
    if (file == 1 || file == 2)
    {
        error_status status;
        uint16_t i = 0;
        
        while (i < len)
        {
            do
            {
                status = usb_vcp_send_data(&usb_core_dev, (uint8_t *)&ptr[i], 1);
            } while (status == ERROR);
            i++;
        }
        
        return len;
    }
    
    return -1;
}

/**
 * @brief 实现__io_putchar函数，确保printf正常工作
 */
int __io_putchar(int ch)
{
    error_status status;
    do
    {
        status = usb_vcp_send_data(&usb_core_dev, (uint8_t *)&ch, 1);
    } while (status == ERROR);
    return ch;
}

