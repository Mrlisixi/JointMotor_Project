#ifndef USB_PRINTF_H
#define USB_PRINTF_H

#include <stdint.h>

/**
 * @brief 将浮点数转换为字符串
 * @param value 浮点数
 * @param buf 输出缓冲区
 * @param precision 小数位数
 * @return 转换后的字符串长度
 */
int float_to_string(float value, char *buf, int precision);

/**
 * @brief 发送字符串到USB
 * @param str 字符串
 * @return 发送的字符数
 */
int usb_send_string(const char *str);

/**
 * @brief 发送浮点数到USB
 * @param value 浮点数
 * @param precision 小数位数
 * @return 发送的字符数
 */
int usb_send_float(float value, int precision);

#endif /* USB_PRINTF_H */
