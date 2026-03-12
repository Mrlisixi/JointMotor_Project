#include "usb_cdc.h"
#include <string.h>
#include "usbd_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "usb_std.h"

/* 外部变量 */
extern usbd_core_type usb_core_dev;

cdc_handle_t cdc_dev = {
  .rx_len = 0,
  .tx_len = 0,
  .is_connected = 0,
  .baudrate = 115200
};

void cdc_init(void)
{
  /* USB初始化已经在usb_app.c中完成 */
}

void cdc_process(void)
{
  /* USB处理已经在usb_app.c中完成 */
}

uint16_t cdc_send_data(uint8_t *data, uint16_t len)
{
  if (!cdc_dev.is_connected)
  {
    return 0;
  }
  
  if (len > CDC_TX_BUFFER_SIZE)
  {
    len = CDC_TX_BUFFER_SIZE;
  }
  
  return usb_vcp_send_data(&usb_core_dev, data, len);
}

uint16_t cdc_receive_data(uint8_t *data, uint16_t len)
{
  if (cdc_dev.rx_len == 0)
  {
    return 0;
  }
  
  if (len > cdc_dev.rx_len)
  {
    len = cdc_dev.rx_len;
  }
  
  /* 使用标准库的memcpy函数 */
  memcpy(data, cdc_dev.rx_buffer, len);
  
  // 从缓冲区中移除已读取的数据
  if (len < cdc_dev.rx_len)
  {
    memmove(cdc_dev.rx_buffer, cdc_dev.rx_buffer + len, cdc_dev.rx_len - len);
  }
  cdc_dev.rx_len -= len;
  
  return len;
}

uint8_t cdc_is_connected(void)
{
  return cdc_dev.is_connected;
}

/* USB CDC回调函数 */
void cdc_class_data_received(uint8_t *data, uint16_t len)
{
  if (len > CDC_RX_BUFFER_SIZE - cdc_dev.rx_len)
  {
    len = CDC_RX_BUFFER_SIZE - cdc_dev.rx_len;
  }
  
  /* 使用标准库的memcpy函数追加数据 */
  memcpy(cdc_dev.rx_buffer + cdc_dev.rx_len, data, len);
  cdc_dev.rx_len += len;
}

void cdc_class_set_line_coding(uint8_t *line_coding)
{
  /* 处理线路编码设置 */
  linecoding_type *lc = (linecoding_type *)line_coding;
  cdc_dev.baudrate = lc->bitrate;
}

void cdc_class_set_control_line_state(uint8_t state)
{
  /* 处理控制线状态 */
}

void cdc_class_connection_changed(uint8_t connected)
{
  cdc_dev.is_connected = connected;
}