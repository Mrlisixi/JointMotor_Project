# 参数字典与Modbus通信协议文档

## 1. 协议概述

本协议基于Modbus RTU实现，用于上位机与设备之间的通信，支持参数的监控与修改。Modbus是一种工业标准的通信协议，广泛应用于工业自动化领域，具有良好的兼容性和可靠性。

## 2. 参数字典概述

参数字典是设备内部所有可配置和监控参数的集合，通过Modbus协议可以访问和修改这些参数。参数字典定义在 `Core/inc/motor_params.h` 文件中，包含了以下主要部分：

- **基本参数**：电机极对数、母线电压、最大电流等
- **PID参数**：位置环、速度环、电流环的PID参数
- **校准参数**：相位偏移、电流偏移等
- **监控参数**：相电流、母线电压、温度等
- **FOC控制参数**：控制模式、目标值、实际值等
- **保护参数**：过流、欠压、过压、过热保护设置
- **CANopen参数**：设备信息、通信配置等
- **DS402参数**：驱动配置、操作模式等
- **制造商特定参数**：各种高级功能配置

## 3. 协议特点

- **标准协议**：采用广泛支持的Modbus RTU协议
- **可靠性**：包含完整的CRC16校验
- **灵活性**：支持多种数据操作方式
- **可扩展性**：易于添加新的寄存器和功能
- **调试方便**：可使用现成的Modbus调试工具
- **全面的参数覆盖**：支持访问和修改设备的所有关键参数

## 4. 帧格式

Modbus RTU帧格式如下：

| 字段 | 长度（字节） | 说明 |
|------|------------|------|
| 从站地址 | 1 | 设备地址，默认为1 |
| 功能码 | 1 | 操作类型 |
| 数据 | 可变 | 命令相关的数据 |
| CRC校验 | 2 | 循环冗余校验 |

## 5. 支持的功能码

| 功能码 | 功能 | 说明 |
|--------|------|------|
| 0x03 | 读取保持寄存器 | 读取可读写的参数 |
| 0x04 | 读取输入寄存器 | 读取只读的状态参数 |
| 0x06 | 写入单个寄存器 | 写入单个参数 |
| 0x10 | 写入多个寄存器 | 批量写入多个参数 |

## 6. 寄存器映射

### 6.1 保持寄存器（可读写）

| 寄存器地址 | 功能 | 数据类型 | 缩放因子 | 范围 | 对应参数字典ID |
|------------|------|----------|----------|------|----------------|
| 0x0000 | 目标速度 | int16 | 0.001 | -3000.0 ~ 3000.0 | PARAM_ID_TARGET_VELOCITY_MOTOR (0x6464) |
| 0x0002 | 比例增益 (KP) | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_VELOCITY_KP (0x646C) |
| 0x0004 | 积分增益 (KI) | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_VELOCITY_KI (0x646D) |
| 0x0006 | 最大速度 | int16 | 0.001 | 0.0 ~ 3000.0 | PARAM_ID_MAX_VELOCITY (0x6404) |
| 0x0008 | 最大电流 | int16 | 0.001 | 0.0 ~ 10.0 | PARAM_ID_MAX_CURRENT (0x6403) |
| 0x000A | 加速度 | int16 | 0.001 | 0.0 ~ 1000.0 | PARAM_ID_TRAPEZOIDAL_ACCELERATION (0x6083) |
| 0x000C | 减速度 | int16 | 0.001 | 0.0 ~ 1000.0 | PARAM_ID_TRAPEZOIDAL_DECELERATION (0x6084) |
| 0x000E | 位置环KP | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_POSITION_KP (0x6469) |
| 0x0010 | 位置环KI | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_POSITION_KI (0x646A) |
| 0x0012 | 电流环KP | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_CURRENT_KP (0x646F) |
| 0x0014 | 电流环KI | int16 | 0.001 | 0.0 ~ 100.0 | PARAM_ID_CURRENT_KI (0x6470) |

### 6.2 输入寄存器（只读）

| 寄存器地址 | 功能 | 数据类型 | 缩放因子 | 范围 | 对应参数字典ID |
|------------|------|----------|----------|------|----------------|
| 0x1000 | 实际速度 | int16 | 0.001 | -3000.0 ~ 3000.0 | PARAM_ID_CURRENT_VELOCITY (0x6467) |
| 0x1002 | 实际电流 | int16 | 0.001 | 0.0 ~ 10.0 | PARAM_ID_ACTUAL_CURRENT (0x6078) |
| 0x1004 | 母线电压 | int16 | 0.001 | 0.0 ~ 30.0 | PARAM_ID_BUS_VOLTAGE_MON (0x6453) |
| 0x1006 | 温度 | int16 | 0.001 | -40.0 ~ 125.0 | PARAM_ID_TEMPERATURE_MON (0x6454) |
| 0x1008 | 错误码 | int16 | 1 | 0 ~ 65535 | PARAM_ID_ERROR_CODE (0x6030) |
| 0x100A | U相电流 | int16 | 0.001 | -10.0 ~ 10.0 | PARAM_ID_U_PHASE_CURRENT (0x6450) |
| 0x100C | V相电流 | int16 | 0.001 | -10.0 ~ 10.0 | PARAM_ID_V_PHASE_CURRENT (0x6451) |
| 0x100E | W相电流 | int16 | 0.001 | -10.0 ~ 10.0 | PARAM_ID_W_PHASE_CURRENT (0x6452) |
| 0x1010 | 当前位置 | int32 | 0.001 | -2^31 ~ 2^31-1 | PARAM_ID_CURRENT_POSITION (0x6466) |
| 0x1012 | 目标位置 | int32 | 0.001 | -2^31 ~ 2^31-1 | PARAM_ID_TARGET_POSITION (0x6463) |

## 7. 数据编码

- **浮点数**：使用16位整数表示，缩放因子为0.001
  - 例如：1.0 表示为 1000
  - 例如：-0.5 表示为 -500
- **32位整数**：使用两个连续的16位寄存器表示，采用大端序
  - 例如：0x12345678 表示为 0x1234 和 0x5678
- **CRC校验**：使用标准的Modbus CRC16算法

## 8. 参数字典结构

参数字典采用分层结构，主要包含以下结构体：

### 8.1 基本参数结构体 (motor_basic_params_t)
- 极对数、母线电压、最大电流、最大速度等基本配置

### 8.2 PID参数结构体 (motor_pid_params_t)
- 位置环、速度环、电流环的PID参数

### 8.3 监控参数结构体 (motor_monitor_params_t)
- 相电流、母线电压、温度等实时监控数据

### 8.4 FOC控制参数结构体 (motor_foc_params_t)
- 控制模式、目标值、实际值、角度信息等

### 8.5 保护参数结构体 (motor_protection_params_t)
- 各种保护阈值和使能状态

### 8.6 DS402参数结构体 (motor_ds402_params_t)
- 符合DS402标准的驱动配置参数

### 8.7 制造商特定参数结构体 (motor_mfg_params_t)
- 各种高级功能和调试参数

## 9. 示例

### 9.1 读取保持寄存器

**请求**（读取目标速度和KP增益）：
```
01 03 00 00 00 03 05 CB
```
- 从站地址：01
- 功能码：03（读取保持寄存器）
- 起始地址：00 00
- 数量：00 03（读取3个寄存器）
- CRC校验：05 CB

**响应**：
```
01 03 06 03 E8 00 C8 00 00 85 4A
```
- 从站地址：01
- 功能码：03（读取保持寄存器）
- 字节数：06
- 数据：03 E8（1000，目标速度1.0） 00 C8（200，KP增益0.2） 00 00（KI增益0.0）
- CRC校验：85 4A

### 9.2 写入单个寄存器

**请求**（设置目标速度为2.5）：
```
01 06 00 00 09 C4 31 88
```
- 从站地址：01
- 功能码：06（写入单个寄存器）
- 地址：00 00
- 值：09 C4（2500，对应2.5）
- CRC校验：31 88

**响应**：
```
01 06 00 00 09 C4 31 88
```
- 从站地址：01
- 功能码：06（写入单个寄存器）
- 地址：00 00
- 值：09 C4（2500，对应2.5）
- CRC校验：31 88

### 9.3 读取输入寄存器

**请求**（读取实际速度和电流）：
```
01 04 10 00 00 02 60 4A
```
- 从站地址：01
- 功能码：04（读取输入寄存器）
- 起始地址：10 00（4096）
- 数量：00 02（读取2个寄存器）
- CRC校验：60 4A

**响应**：
```
01 04 04 03 E8 00 64 8E 78
```
- 从站地址：01
- 功能码：04（读取输入寄存器）
- 字节数：04
- 数据：03 E8（1000，实际速度1.0） 00 64（100，实际电流0.1）
- CRC校验：8E 78

### 9.4 写入多个寄存器

**请求**（设置KP和KI增益）：
```
01 10 00 02 00 02 04 00 C8 00 64 4F 6A
```
- 从站地址：01
- 功能码：10（写入多个寄存器）
- 起始地址：00 02
- 数量：00 02（写入2个寄存器）
- 字节数：04
- 数据：00 C8（200，KP增益0.2） 00 64（100，KI增益0.1）
- CRC校验：4F 6A

**响应**：
```
01 10 00 02 00 02 79 39
```
- 从站地址：01
- 功能码：10（写入多个寄存器）
- 起始地址：00 02
- 数量：00 02
- CRC校验：79 39

## 10. 实现说明

- **协议处理**：在 `Common/src/modbus_protocol.c` 中实现
- **数据收发**：通过USB CDC接口实现
- **FreeRTOS集成**：在 `project/src/freertos_app.c` 的Debug_Task中处理协议
- **寄存器映射**：在 `Common/inc/modbus_protocol.h` 中定义
- **参数字典**：在 `Core/inc/motor_params.h` 中定义
- **参数访问**：通过 `motor_params_update()` 和 `motor_params_get()` 函数访问参数字典
- **环形队列**：在 `Common/src/modbus_protocol.c` 中实现，用于高效处理USB数据

### 10.1 环形队列实现

环形队列（Ring Buffer）是一种高效的FIFO数据结构，用于处理USB数据的接收和处理。实现特点包括：

- **缓冲区大小**：512字节，足够处理Modbus帧
- **线程安全**：适用于FreeRTOS环境
- **溢出处理**：当缓冲区满时，自动丢弃最旧的数据
- **批量操作**：支持批量弹出数据，提高处理效率
- **零拷贝**：使用内存复制减少数据移动

核心函数：
- `ring_buffer_push(uint8_t data)`：压入数据到队列
- `ring_buffer_pop()`：弹出队列头部数据
- `ring_buffer_pop_n(uint16_t n)`：批量弹出n个数据
- `ring_buffer_clear()`：清空队列
- `ring_buffer_count()`：获取队列中的数据长度
- `ring_buffer_copy(uint8_t *dest, uint16_t len)`：复制队列数据到目标缓冲区

### 10.2 Modbus协议实现

Modbus协议实现采用标准的Modbus RTU格式，支持以下功能：

- **功能码支持**：0x03（读取保持寄存器）、0x04（读取输入寄存器）、0x06（写入单个寄存器）、0x10（写入多个寄存器）
- **CRC校验**：标准Modbus CRC16算法
- **错误处理**：完整的异常响应机制
- **参数访问**：直接映射到参数字典，支持所有参数的读写
- **数据编码**：16位整数表示浮点数，缩放因子为0.001

### 10.3 直接参数访问

Modbus实现采用直接参数访问方式，无需寄存器地址映射表，具体特点：

- **地址直接对应**：Modbus寄存器地址直接对应参数字典的参数ID
- **全参数支持**：支持访问参数字典中的所有参数
- **动态映射**：无需预定义寄存器映射表，减少维护成本
- **扩展性**：添加新参数后自动支持，无需修改Modbus代码

例如：
- 读取参数ID为0x6401（极对数）的保持寄存器：发送命令 `01 03 64 01 00 01 CRC`
- 写入参数ID为0x6464（目标速度）的保持寄存器：发送命令 `01 06 64 64 03 E8 CRC`（设置为1.0）

## 11. 上位机开发建议

### 11.1 软件选择
- **专业工具**：Modbus Poll（Windows）、QModMaster（跨平台）
- **编程库**：libmodbus（C/C++）、pymodbus（Python）、NModbus（.NET）

### 11.2 通信流程
1. 建立USB串口连接
2. 配置串口参数（波特率115200，8N1）
3. 发送Modbus命令
4. 接收并解析响应
5. 处理数据和错误

### 11.3 错误处理
- 检查CRC校验
- 处理异常响应（功能码最高位为1）
- 实现超时机制
- 处理连接断开情况

### 11.4 参数管理
- 维护寄存器映射表
- 实现数据类型转换（处理缩放因子）
- 提供用户友好的参数编辑界面
- 实现参数范围检查
- 支持批量参数导入/导出

### 11.5 性能优化
- 使用批量命令减少通信次数
- 合理设置通信频率
- 实现数据缓存机制
- 避免频繁写入参数
- 对实时性要求高的参数使用快速更新

## 12. 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|---------|----------|
| 无响应 | USB连接问题 | 检查USB线缆和驱动 |
| CRC错误 | 数据传输错误 | 重新发送命令 |
| 非法数据地址 | 寄存器地址错误 | 检查寄存器映射表 |
| 非法数据值 | 值超出范围 | 检查参数范围 |
| 设备忙 | 处理其他任务 | 稍后重试 |
| 写入失败 | 权限限制 | 检查参数是否可写 |
| 读取值异常 | 传感器故障 | 检查硬件连接 |

## 13. 测试步骤

1. **准备工作**：
   - 安装Modbus Poll或其他Modbus调试工具
   - 连接开发板到电脑
   - 烧录包含Modbus协议的固件

2. **基本测试**：
   - 读取保持寄存器（功能码03）
   - 写入单个寄存器（功能码06）
   - 读取输入寄存器（功能码04）
   - 写入多个寄存器（功能码10）

3. **功能测试**：
   - 测试速度控制：设置目标速度，读取实际速度
   - 测试PID参数调整：修改KP/KI值，观察响应
   - 测试状态监控：读取电流、电压、温度等参数
   - 测试错误处理：故意写入超出范围的值，观察异常响应

4. **高级测试**：
   - 批量参数配置：使用功能码10同时修改多个参数
   - 连续监控：以一定频率读取输入寄存器，观察数据变化
   - 边界测试：测试参数的最小/最大值

## 14. 版本历史

| 版本 | 日期 | 变更内容 |
|------|------|----------|
| 2.1 | 2026-03-08 | 完善参数字典文档，添加详细的寄存器映射 |
| 2.0 | 2026-03-07 | 迁移到Modbus RTU协议 |
| 1.0 | 2026-03-06 | 初始版本（USB CDC自定义协议） |

## 15. 附录：Modbus CRC16算法

```c
uint16_t modbus_calculate_crc(uint8_t *data, uint16_t len)
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
```

## 16. 附录：寄存器地址映射表

### 保持寄存器
| 地址 | 名称 | 描述 | 对应参数字典ID |
|------|------|------|----------------|
| 0x0000 | REG_TARGET_VELOCITY | 目标速度 | PARAM_ID_TARGET_VELOCITY_MOTOR (0x6464) |
| 0x0002 | REG_KP_GAIN | 比例增益 | PARAM_ID_VELOCITY_KP (0x646C) |
| 0x0004 | REG_KI_GAIN | 积分增益 | PARAM_ID_VELOCITY_KI (0x646D) |
| 0x0006 | REG_MAX_VELOCITY | 最大速度 | PARAM_ID_MAX_VELOCITY (0x6404) |
| 0x0008 | REG_MAX_CURRENT | 最大电流 | PARAM_ID_MAX_CURRENT (0x6403) |
| 0x000A | REG_ACCELERATION | 加速度 | PARAM_ID_TRAPEZOIDAL_ACCELERATION (0x6083) |
| 0x000C | REG_DECELERATION | 减速度 | PARAM_ID_TRAPEZOIDAL_DECELERATION (0x6084) |
| 0x000E | REG_POSITION_KP | 位置环KP | PARAM_ID_POSITION_KP (0x6469) |
| 0x0010 | REG_POSITION_KI | 位置环KI | PARAM_ID_POSITION_KI (0x646A) |
| 0x0012 | REG_CURRENT_KP | 电流环KP | PARAM_ID_CURRENT_KP (0x646F) |
| 0x0014 | REG_CURRENT_KI | 电流环KI | PARAM_ID_CURRENT_KI (0x6470) |

### 输入寄存器
| 地址 | 名称 | 描述 | 对应参数字典ID |
|------|------|------|----------------|
| 0x1000 | REG_ACTUAL_VELOCITY | 实际速度 | PARAM_ID_CURRENT_VELOCITY (0x6467) |
| 0x1002 | REG_ACTUAL_CURRENT | 实际电流 | PARAM_ID_ACTUAL_CURRENT (0x6078) |
| 0x1004 | REG_BUS_VOLTAGE | 母线电压 | PARAM_ID_BUS_VOLTAGE_MON (0x6453) |
| 0x1006 | REG_TEMPERATURE | 温度 | PARAM_ID_TEMPERATURE_MON (0x6454) |
| 0x1008 | REG_ERROR_CODE | 错误码 | PARAM_ID_ERROR_CODE (0x6030) |
| 0x100A | REG_U_PHASE_CURRENT | U相电流 | PARAM_ID_U_PHASE_CURRENT (0x6450) |
| 0x100C | REG_V_PHASE_CURRENT | V相电流 | PARAM_ID_V_PHASE_CURRENT (0x6451) |
| 0x100E | REG_W_PHASE_CURRENT | W相电流 | PARAM_ID_W_PHASE_CURRENT (0x6452) |
| 0x1010 | REG_CURRENT_POSITION | 当前位置 | PARAM_ID_CURRENT_POSITION (0x6466) |
| 0x1012 | REG_TARGET_POSITION | 目标位置 | PARAM_ID_TARGET_POSITION (0x6463) |

## 17. 附录：参数字典结构说明

### 17.1 基本参数
| 参数ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| 0x6401 | POLE_PAIRS | uint16 | 电机极对数 |
| 0x6402 | BUS_VOLTAGE | float | 母线电压 |
| 0x6403 | MAX_CURRENT | float | 最大电流 |
| 0x6404 | MAX_VELOCITY | float | 最大速度 |
| 0x6405 | MAX_TORQUE | float | 最大转矩 |
| 0x6406 | CONTROL_DT | float | 控制周期 |
| 0x6407 | PWM_FREQUENCY | float | PWM频率 |

### 17.2 PID参数
| 参数ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| 0x6469 | POSITION_KP | float | 位置环比例增益 |
| 0x646A | POSITION_KI | float | 位置环积分增益 |
| 0x646B | POSITION_KD | float | 位置环微分增益 |
| 0x646C | VELOCITY_KP | float | 速度环比例增益 |
| 0x646D | VELOCITY_KI | float | 速度环积分增益 |
| 0x646E | VELOCITY_KD | float | 速度环微分增益 |
| 0x646F | CURRENT_KP | float | 电流环比例增益 |
| 0x6470 | CURRENT_KI | float | 电流环积分增益 |
| 0x6471 | CURRENT_KD | float | 电流环微分增益 |

### 17.3 监控参数
| 参数ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| 0x6450 | U_PHASE_CURRENT | float | U相电流 |
| 0x6451 | V_PHASE_CURRENT | float | V相电流 |
| 0x6452 | W_PHASE_CURRENT | float | W相电流 |
| 0x6453 | BUS_VOLTAGE_MON | float | 母线电压 |
| 0x6454 | TEMPERATURE_MON | float | 温度 |

### 17.4 FOC控制参数
| 参数ID | 名称 | 类型 | 说明 |
|--------|------|------|------|
| 0x6462 | CONTROL_MODE | uint8 | 控制模式 |
| 0x6463 | TARGET_POSITION | float | 目标位置 |
| 0x6464 | TARGET_VELOCITY_MOTOR | float | 目标速度 |
| 0x6465 | TARGET_TORQUE | float | 目标转矩 |
| 0x6466 | CURRENT_POSITION | float | 当前位置 |
| 0x6467 | CURRENT_VELOCITY | float | 当前速度 |
| 0x6468 | CURRENT_TORQUE | float | 当前转矩 |
| 0x6472 | FOC_ENABLED | uint8 | FOC使能状态 |