#ifndef __USB_CDC_H__
#define __USB_CDC_H__

#include "at32f403a_407.h"

#define CDC_RX_BUFFER_SIZE 256
#define CDC_TX_BUFFER_SIZE 256

typedef struct
{
  uint8_t rx_buffer[CDC_RX_BUFFER_SIZE];
  uint16_t rx_len;
  uint8_t tx_buffer[CDC_TX_BUFFER_SIZE];
  uint16_t tx_len;
  uint8_t is_connected;
  uint32_t baudrate;
} cdc_handle_t;

extern cdc_handle_t cdc_dev;

void cdc_init(void);
void cdc_process(void);
uint16_t cdc_send_data(uint8_t *data, uint16_t len);
uint16_t cdc_receive_data(uint8_t *data, uint16_t len);
uint8_t cdc_is_connected(void);

#endif /* __USB_CDC_H__ */