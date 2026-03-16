# JointMotor_Project

## 项目简介

JointMotor_Project 是一个基于 AT32F403A 微控制器的关节电机控制项目，实现了对三相电流、电压和MOS管温度的监控功能。

## 主要功能

- 电机控制：实现了六步换向控制算法
- 电流监控：通过1mΩ分流电阻和RS724XQ运算放大器实现电流采样
- 电压监控：通过18K+1K电阻分压实现电压采样
- 温度监控：通过10K F3950热敏电阻实现温度采样
- 数据输出：通过串口输出监控数据

## 硬件配置

- 微控制器：AT32F403ACGT7
- 电流传感器：1mΩ分流电阻 + RS724XQ运算放大器
- 电压传感器：18K+1K电阻分压
- 温度传感器：10K F3950热敏电阻
- 参考电压：1.65V

## 软件架构

- FreeRTOS：实时操作系统
- 模块化设计：硬件抽象层与应用逻辑分离
- 监控任务：每100ms获取一次监控数据

## 目录结构

```
JointMotor_Project/
├── AT32WorkBench/          # AT32工作台配置文件
│   ├── project/            # 项目文件
│   ├── libraries/          # 库文件
│   └── middlewares/        # 中间件
├── Motor_Control/          # 电机控制模块
│   ├── inc/                # 头文件
│   └── src/                # 源文件
├── Readme.md               # 项目说明
└── Makefile                # 构建文件
```

## 构建与运行

1. 使用 ARM GCC 工具链编译项目
2. 通过 JTAG/SWD 下载到开发板
3. 打开串口终端，波特率设置为 115200
4. 观察监控数据输出

## 监控数据格式

```
Current A: X.XX A, Current B: X.XX A, Current C: X.XX A
Voltage A: X.XX V, Voltage B: X.XX V, Voltage C: X.XX V
DC Link Voltage: X.XX V, MOS Temperature: X.XX °C
```

## 注意事项

- 确保参考电压稳定在1.65V
- 电机运行时注意散热
- 定期检查传感器连接

## 许可证

MIT License
