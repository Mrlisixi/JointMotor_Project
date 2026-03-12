#include "modbus_protocol.h"
#include <string.h>
#include "usbd_core.h"
#include "cdc_class.h"

/* 外部变量 */
extern motor_params_t motor;
extern usbd_core_type usb_core_dev;

/* 内部变量 - 环形队列实现 */
#define RX_BUFFER_SIZE 512
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_head = 0;  // 队首指针
static uint16_t rx_tail = 0;  // 队尾指针
static uint16_t rx_count = 0; // 队列中的数据长度

/* 环形队列函数声明 */
static uint8_t ring_buffer_pop(void);
static void ring_buffer_push(uint8_t data);
static void ring_buffer_clear(void);
static uint16_t ring_buffer_count(void);
static void ring_buffer_copy(uint8_t *dest, uint16_t len);
static void ring_buffer_pop_n(uint16_t n);

/* 环形队列函数 */
static void ring_buffer_push(uint8_t data)
{
  if (rx_count < RX_BUFFER_SIZE)
  {
    rx_buffer[rx_tail] = data;
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    rx_count++;
  }
  else
  {
    // 缓冲区满，丢弃 oldest数据
    ring_buffer_pop();
    ring_buffer_push(data);
  }
}

static uint8_t ring_buffer_pop(void)
{
  uint8_t data = 0;
  if (rx_count > 0)
  {
    data = rx_buffer[rx_head];
    rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
    rx_count--;
  }
  return data;
}

static void ring_buffer_clear(void)
{
  rx_head = 0;
  rx_tail = 0;
  rx_count = 0;
}

static uint16_t ring_buffer_count(void)
{
  return rx_count;
}

static void ring_buffer_copy(uint8_t *dest, uint16_t len)
{
  memset(dest, 0, len); // 先清空目标缓冲区
  uint16_t copy_len = (len < rx_count) ? len : rx_count;
  for (uint16_t i = 0; i < copy_len; i++)
  {
    uint16_t index = (rx_head + i) % RX_BUFFER_SIZE;
    dest[i] = rx_buffer[index];
  }
}

static void ring_buffer_pop_n(uint16_t n)
{
  for (uint16_t i = 0; i < n && rx_count > 0; i++)
  {
    ring_buffer_pop();
  }
}

/* CRC16计算函数 */
static uint16_t modbus_calculate_crc(uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  uint8_t i;
  
  while (len--)
  {
    crc ^= *data++;
    for (i = 0; i < 8; i++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

/* 初始化Modbus协议 */
void modbus_protocol_init(void)
{
  /* USB初始化已经在usb_app.c中完成 */
}

/* 处理Modbus协议 */
void modbus_protocol_process(void)
{
  /* 直接从USB核心获取数据 */
  uint8_t usb_rx_buffer[USBD_CDC_OUT_MAXPACKET_SIZE];
  uint16_t recv_len = usb_vcp_get_rxdata(&usb_core_dev, usb_rx_buffer);
  
  if (recv_len > 0)
  {
    /* 将数据压入环形队列 */
    for (uint16_t i = 0; i < recv_len; i++)
    {
      ring_buffer_push(usb_rx_buffer[i]);
    }
    
    /* 处理接收到的帧 */
    while (ring_buffer_count() >= 4) /* 最小帧长度：从站地址1 + 功能码1 + CRC2 */
    {
      /* 复制当前队列数据到临时缓冲区进行处理 */
      uint8_t temp_buffer[RX_BUFFER_SIZE];
      uint16_t temp_len = ring_buffer_count();
      ring_buffer_copy(temp_buffer, temp_len);
      
      modbus_frame_t *frame = (modbus_frame_t *)temp_buffer;
      
      /* 检查从站地址 */
      if (frame->slave_address != MODBUS_SLAVE_ADDRESS)
      {
        /* 从站地址错误，丢弃数据 */
        ring_buffer_pop();
        continue;
      }
      
      /* 计算帧长度 */
      uint16_t frame_len = 4; /* 最小帧长度 */
      
      switch (frame->function_code)
      {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        case MODBUS_FC_READ_INPUT_REGISTERS:
          frame_len = 8; /* 从站地址1 + 功能码1 + 起始地址2 + 数量2 + CRC2 */
          break;
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
          frame_len = 8; /* 从站地址1 + 功能码1 + 地址2 + 值2 + CRC2 */
          break;
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
          if (temp_len >= 7) /* 至少需要7字节才能获取字节数 */
          {
            frame_len = 7 + temp_buffer[6]; /* 从站地址1 + 功能码1 + 起始地址2 + 数量2 + 字节数1 + 数据N + CRC2 */
          }
          break;
        default:
          /* 非法功能码 */
          modbus_send_exception_response(frame->function_code, MODBUS_ERR_ILLEGAL_FUNCTION);
          ring_buffer_pop();
          continue;
      }
      
      if (temp_len < frame_len)
      {
        /* 数据不完整，等待更多数据 */
        break;
      }
      
      /* 检查CRC */
      uint16_t calculated_crc = modbus_calculate_crc(temp_buffer, frame_len - 2);
      uint16_t received_crc = ((uint16_t)temp_buffer[frame_len - 2]) | ((uint16_t)temp_buffer[frame_len - 1] << 8);
      
      if (calculated_crc != received_crc)
      {
        /* CRC错误，丢弃数据 */
        ring_buffer_pop();
        continue;
      }
      
      /* 处理命令 */
      switch (frame->function_code)
      {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
        {
          // 手动解析大端序数据
          uint16_t start_address = (temp_buffer[1] << 8) | temp_buffer[2];
          uint16_t quantity = (temp_buffer[3] << 8) | temp_buffer[4];
          
          if (quantity > 64)
          {
            modbus_send_exception_response(frame->function_code, MODBUS_ERR_ILLEGAL_DATA_VALUE);
            // 从队列中移除整个帧
            ring_buffer_pop_n(frame_len);
            break;
          }
          
          // 准备响应数据（大端序）
          uint8_t response_data[1 + 64 * 2]; // 字节数 + 最多64个寄存器
          response_data[0] = quantity * 2;
          
          for (uint16_t i = 0; i < quantity; i++)
          {
            uint16_t param_id = start_address + i;
            
            float value = motor_params_get(&motor, param_id);
            
            /* 将浮点数转换为16位整数（缩放1000倍） */
            int16_t scaled_value = (int16_t)(value * 1000.0f);
            
            // 大端序存储
            response_data[1 + i * 2] = (scaled_value >> 8) & 0xFF;
            response_data[2 + i * 2] = scaled_value & 0xFF;
          }
          
          modbus_send_response(frame->function_code, response_data, response_data[0] + 1);
          
          // 从队列中移除整个帧
          ring_buffer_pop_n(frame_len);
          break;
        }
        case MODBUS_FC_READ_INPUT_REGISTERS:
        {
          // 手动解析大端序数据
          uint16_t start_address = (temp_buffer[1] << 8) | temp_buffer[2];
          uint16_t quantity = (temp_buffer[3] << 8) | temp_buffer[4];
          
          if (quantity > 64)
          {
            modbus_send_exception_response(frame->function_code, MODBUS_ERR_ILLEGAL_DATA_VALUE);
            // 从队列中移除整个帧
            ring_buffer_pop_n(frame_len);
            break;
          }
          
          // 准备响应数据（大端序）
          uint8_t response_data[1 + 64 * 2]; // 字节数 + 最多64个寄存器
          response_data[0] = quantity * 2;
          
          for (uint16_t i = 0; i < quantity; i++)
          {
            uint16_t param_id = start_address + i;
            
            float value = motor_params_get(&motor, param_id);
            
            /* 将浮点数转换为16位整数（缩放1000倍） */
            int16_t scaled_value = (int16_t)(value * 1000.0f);
            
            // 大端序存储
            response_data[1 + i * 2] = (scaled_value >> 8) & 0xFF;
            response_data[2 + i * 2] = scaled_value & 0xFF;
          }
          
          modbus_send_response(frame->function_code, response_data, response_data[0] + 1);
          
          // 从队列中移除整个帧
          ring_buffer_pop_n(frame_len);
          break;
        }
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
        {
          // 手动解析大端序数据
          uint16_t param_id = (temp_buffer[1] << 8) | temp_buffer[2];
          
          int16_t scaled_value = (temp_buffer[3] << 8) | temp_buffer[4];
          float value = scaled_value / 1000.0f;
          
          // 直接使用参数字典更新函数
          motor_params_update(&motor, param_id, value);
          
          // 准备响应数据（大端序）
          uint8_t response_data[4];
          response_data[0] = (param_id >> 8) & 0xFF;
          response_data[1] = param_id & 0xFF;
          response_data[2] = (scaled_value >> 8) & 0xFF;
          response_data[3] = scaled_value & 0xFF;
          
          modbus_send_response(frame->function_code, response_data, 4);
          
          // 从队列中移除整个帧
          ring_buffer_pop_n(frame_len);
          break;
        }
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        {
          // 手动解析大端序数据
          uint16_t start_address = (temp_buffer[1] << 8) | temp_buffer[2];
          uint16_t quantity = (temp_buffer[3] << 8) | temp_buffer[4];
          
          if (quantity > 64)
          {
            modbus_send_exception_response(frame->function_code, MODBUS_ERR_ILLEGAL_DATA_VALUE);
            // 从队列中移除整个帧
            ring_buffer_pop_n(frame_len);
            break;
          }
          
          for (uint16_t i = 0; i < quantity; i++)
          {
            uint16_t param_id = start_address + i;
            
            // 手动解析大端序数据
            int16_t scaled_value = (temp_buffer[6 + i * 2] << 8) | temp_buffer[7 + i * 2];
            float value = scaled_value / 1000.0f;
            
            // 直接使用参数字典更新函数
            motor_params_update(&motor, param_id, value);
          }
          
          // 准备响应数据（大端序）
          uint8_t response_data[4];
          response_data[0] = (start_address >> 8) & 0xFF;
          response_data[1] = start_address & 0xFF;
          response_data[2] = (quantity >> 8) & 0xFF;
          response_data[3] = quantity & 0xFF;
          
          modbus_send_response(frame->function_code, response_data, 4);
          
          // 从队列中移除整个帧
          ring_buffer_pop_n(frame_len);
          break;
        }
        default:
          // 从队列中移除整个帧
          ring_buffer_pop_n(frame_len);
          break;
      }
    }
  }
}

/* 发送响应 */
void modbus_send_response(uint8_t function_code, void *data, uint16_t data_len)
{
  uint8_t response_buffer[256];
  uint16_t response_len = 2; /* 从站地址 + 功能码 */
  
  /* 设置从站地址和功能码 */
  response_buffer[0] = MODBUS_SLAVE_ADDRESS;
  response_buffer[1] = function_code;
  
  /* 复制数据 */
  if (data != NULL && data_len > 0)
  {
    memcpy(response_buffer + 2, data, data_len);
    response_len += data_len;
  }
  
  /* 计算CRC */
  uint16_t crc = modbus_calculate_crc(response_buffer, response_len);
  response_buffer[response_len++] = (uint8_t)(crc & 0xFF);
  response_buffer[response_len++] = (uint8_t)(crc >> 8);
  
  /* 发送数据 */
  usb_vcp_send_data(&usb_core_dev, response_buffer, response_len);
}

/* 发送异常响应 */
void modbus_send_exception_response(uint8_t function_code, uint8_t exception_code)
{
  uint8_t response_buffer[5];
  
  /* 设置从站地址、功能码（最高位置1）和异常码 */
  response_buffer[0] = MODBUS_SLAVE_ADDRESS;
  response_buffer[1] = function_code | 0x80;
  response_buffer[2] = exception_code;
  
  /* 计算CRC */
  uint16_t crc = modbus_calculate_crc(response_buffer, 3);
  response_buffer[3] = (uint8_t)(crc & 0xFF);
  response_buffer[4] = (uint8_t)(crc >> 8);
  
  /* 发送数据 */
  usb_vcp_send_data(&usb_core_dev, response_buffer, 5);
}