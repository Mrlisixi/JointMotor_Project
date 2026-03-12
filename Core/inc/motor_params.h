/* 电机参数配置字典 */
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include "at32f403a_407.h"

/* 参数类型枚举 */
typedef enum {
    PARAM_TYPE_INT = 0,
    PARAM_TYPE_FLOAT = 1,
    PARAM_TYPE_BOOL = 2,
    PARAM_TYPE_STRING = 3
} param_type_t;

/* FOC控制模式枚举 */
typedef enum {
    FOC_CONTROL_MODE_POSITION = 0,
    FOC_CONTROL_MODE_VELOCITY = 1,
    FOC_CONTROL_MODE_TORQUE = 2
} foc_control_mode_t;

/**
  * @brief  电机状态枚举
  */
typedef enum {
    MOTOR_STATE_IDLE = 0,      /* 空闲状态 */
    MOTOR_STATE_INIT = 1,      /* 初始化状态 */
    MOTOR_STATE_RUNNING = 2,   /* 运行状态 */
    MOTOR_STATE_FAULT = 3,     /* 故障状态 */
    MOTOR_STATE_STOPPING = 4   /* 停止状态 */
} motor_state_t;


/* 三相电流结构体 */
typedef struct {
    float u;     // U相电流
    float v;     // V相电流
    float w;     // W相电流
} uvw_t;

/* Alpha-Beta电流结构体 */
typedef struct {
    float alpha; // Alpha轴电流
    float beta;  // Beta轴电流
} alpha_beta_t;

/* DQ电流结构体 */
typedef struct {
    float d;     // D轴电流
    float q;     // Q轴电流
} dq_t;

/* 角度结构体 */
typedef struct {
    float angle; // 电角度
    float sin;   // 角度正弦值
    float cos;   // 角度余弦值
} angle_t;

/* PWM结构体 */
typedef struct {
    float u;     // U相电压
    float v;     // V相电压
    float w;     // W相电压
} pwm_t;

/* 电机基本参数 */
#define MOTOR_POLE_PAIRS 21
#define MOTOR_VBUS_VOLTS 24.0f
#define MOTOR_MAX_CURRENT 16.0f
#define MOTOR_MAX_VELOCITY 516.0f
#define MOTOR_MAX_TORQUE 5.0f

/* 控制参数 */
#define CONTROL_DT 0.0001f
#define FOC_PWM_FREQUENCY 100000.0f

/* ADC 转换系数 */
#define ADC_TO_AMP_FACTOR  (1.0f)
#define ADC_TO_VOLT_FACTOR (0.01f)
#define ADC_TO_TEMP_FACTOR (0.1f)

/* 位置传感器参数 */
#define ENCODER_RESOLUTION 4096
#define ENCODER_CPR (ENCODER_RESOLUTION * 4)

/* PID参数 */
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

/* 基本参数结构体 */
typedef struct {
    uint16_t pole_pairs;
    float bus_voltage;
    float max_current;
    float max_velocity;
    float max_torque;
    float control_dt;
    float pwm_frequency;
    uint16_t encoder_resolution;
    uint16_t encoder_cpr;
    float temp_threshold;
    float temp_hysteresis;
} motor_basic_params_t;

/* PID参数结构体 */
typedef struct {
    pid_params_t position;
    pid_params_t velocity;
    pid_params_t current;
} motor_pid_params_t;

/* 校准参数结构体 */
typedef struct {
    float phase_offset;
    float current_offset[2];
} motor_calib_params_t;

/* 监控参数结构体 */
typedef struct {
    float u_phase_current;
    float v_phase_current;
    float w_phase_current;
    float bus_voltage;
    float temperature;
    uint32_t sample_freq;
} motor_monitor_params_t;

/* FOC控制参数结构体 */
typedef struct {
    uint16_t max_pwm_value;
    uint32_t sample_freq;
    foc_control_mode_t control_mode;
    float target_position;
    float target_velocity;
    float target_torque;
    float current_position;
    float current_velocity;
    float current_torque;
    float kp_pos;
    float ki_pos;
    float kp_vel;
    float ki_vel;
    float kp_current;
    float ki_current;
    float pos_error_integral;
    float vel_error_integral;
    float d_current_error_integral;
    float q_current_error_integral;
    angle_t angle;
    uvw_t current_uvw;
    alpha_beta_t current_alpha_beta;
    dq_t current_dq;
    dq_t voltage_dq;
    pwm_t pwm;
    uint8_t enabled;
    
    /* 无感控制参数 */
    uint8_t sensorless_enabled;
    float sensorless_k1;
    float sensorless_k2;
    float sensorless_k3;
    float sensorless_min_speed;
    float sensorless_max_speed;
    float sensorless_estimator_gain;
    float estimated_speed;
    float estimated_angle;
    dq_t back_emf_est;
} motor_foc_params_t;

/* 保护参数结构体 */
typedef struct {
    float max_current;
    float min_voltage;
    float max_voltage;
    float max_temperature;
    uint8_t overcurrent_prot;
    uint8_t undervolt_prot;
    uint8_t overvolt_prot;
    uint8_t overtemperature_prot;
    uint8_t fault_status;
    uint8_t fault_active;
} motor_protection_params_t;

/* CANopen DS301参数结构体 */
typedef struct {
    uint32_t device_type;
    uint8_t error_register;
    uint8_t error_field[8];
    uint32_t sync_cob_id;
    uint32_t sync_period;
    char device_name[32];
    char hardware_version[32];
    char software_version[32];
    uint8_t node_id;
    uint16_t watchdog_time;
    uint8_t watchdog_factor;
    uint32_t guard_cob_id;
    uint8_t store_params;
    uint8_t restore_params;
    uint32_t emergency_cob_id;
    uint32_t heartbeat_consumer;
    uint16_t heartbeat_producer;
    uint32_t identity_object[5];
} motor_canopen_params_t;

/* 通信参数结构体 */
typedef struct {
    uint32_t rpdo1_comm[4];
    uint32_t rpdo2_comm[4];
    uint32_t rpdo3_comm[4];
    uint32_t rpdo4_comm[4];
    uint32_t rpdo5_comm[4];
    uint32_t rpdo6_comm[4];
    uint32_t rpdo7_comm[4];
    uint32_t rpdo8_comm[4];
    uint32_t rpdo1_map[8];
    uint32_t rpdo2_map[8];
    uint32_t rpdo3_map[8];
    uint32_t rpdo4_map[8];
    uint32_t rpdo5_map[8];
    uint32_t rpdo6_map[8];
    uint32_t rpdo7_map[8];
    uint32_t rpdo8_map[8];
    uint32_t tpdo1_comm[6];
    uint32_t tpdo2_comm[6];
    uint32_t tpdo3_comm[6];
    uint32_t tpdo4_comm[6];
    uint32_t tpdo5_comm[6];
    uint32_t tpdo6_comm[6];
    uint32_t tpdo7_comm[6];
    uint32_t tpdo8_comm[6];
    uint32_t tpdo1_map[8];
    uint32_t tpdo2_map[8];
    uint32_t tpdo3_map[8];
    uint32_t tpdo4_map[8];
    uint32_t tpdo5_map[8];
    uint32_t tpdo6_map[8];
    uint32_t tpdo7_map[8];
    uint32_t tpdo8_map[8];
    uint32_t nmt_start_stop[8];
    uint32_t nmt_start_stop_ch0[8];
    uint32_t nmt_start_stop_ch1[8];
    uint32_t nmt_start_stop_ch2[8];
    uint32_t nmt_start_stop_ch3[8];
    uint32_t sync_output[13];
    uint32_t sync_input[13];
} motor_comm_params_t;

/* DS402参数结构体 */
typedef struct motor_ds402_params_t {
    int32_t motor_absolute_position;
    uint16_t communication_interrupt_mode;
    uint32_t error_code;
    uint16_t control_word;
    uint16_t status_word;
    uint8_t quick_stop_mode;
    uint8_t shutdown_stop_mode;
    uint8_t disable_stop_mode;
    uint8_t halt_mode;
    uint8_t error_stop_mode;
    uint8_t operation_mode;
    uint8_t current_state;
    uint8_t active_operation_mode;
    int32_t effective_target_position;
    int32_t actual_position_inc;
    int32_t actual_position_pulse;
    int32_t max_following_error;
    int32_t target_position_window;
    uint32_t position_window_time;
    int32_t effective_target_velocity;
    int32_t actual_velocity;
    int16_t target_torque_percent;
    int16_t max_torque_limit_percent;
    int16_t target_current_limit;
    int16_t motor_rated_current;
    int16_t motor_rated_torque;
    int16_t actual_torque;
    int16_t actual_current;
    uint16_t actual_bus_voltage_mv;
    int32_t target_position;
    int32_t home_offset;
    int32_t soft_limit[3];
    uint8_t velocity_position_direction;
    int32_t max_velocity_limit;
    int32_t max_velocity_limit_rpm;
    int32_t trapezoidal_velocity;
    int32_t trapezoidal_acceleration;
    int32_t trapezoidal_deceleration;
    int32_t quick_stop_deceleration;
    uint32_t position_encoder_resolution;
    uint32_t motor_revolutions;
    uint32_t bus_gear_ratio[4];
    uint32_t feed_constant[3];
    uint32_t pos_factor[3];
    uint32_t speed_factor[3];
    uint32_t acc_factor[3];
    uint8_t home_mode;
    uint32_t home_speed[7];
    int32_t home_acceleration;
    uint8_t capture_function;
    uint8_t capture_status;
    int32_t rising_edge_capture_position1;
    int32_t falling_edge_capture_position1;
    int32_t rising_edge_capture_position2;
    int32_t falling_edge_capture_position2;
    uint8_t interpolation_submode;
    uint32_t interpolation_data[3];
    uint32_t interpolation_time[3];
    uint32_t ip_sync_definition[3];
    uint32_t group_ip_data_cfg[7];
    int32_t position_following_error;
    uint32_t current_loop_group2[9];
    uint32_t current_group[43];
    uint32_t power_group[16];
    uint32_t speed_group[44];
    uint32_t position_group[20];
    int32_t effective_target_position2;
    uint16_t input_status;
    uint32_t output_group[2];
    int32_t target_velocity;
} motor_ds402_params_t;

/* 制造商特定参数结构体 */
typedef struct {
    uint32_t power_on_enable;
    uint32_t io_group[40];
    uint32_t input_filter[8];
    uint32_t fast_capture;
    uint32_t rising_capture_count;
    uint32_t rising_capture_data[32];
    uint32_t falling_capture_count;
    uint32_t falling_capture_data[32];
    uint32_t input_delay[7];
    uint32_t capture_compensation[4];
    uint32_t din_control[52];
    uint32_t index_window;
    uint32_t oscilloscope[14];
    uint32_t auto_flip[6];
    uint32_t phase_correction[14];
    uint32_t system_test[13];
    uint32_t special_function[18];
    uint32_t high_voltage_sampling;
    uint32_t adc_group[14];
    uint32_t analog_input[17];
    uint32_t pulse_group2[3];
    uint32_t pulse_group[16];
    uint32_t pulse_group2b[15];
    uint32_t full_closed_loop[10];
    uint32_t notch_filter[9];
    uint32_t notch_filter2[9];
    uint32_t field_weakening[9];
    uint32_t filter_extension[12];
    uint32_t online_tuning[14];
    uint32_t error_mask;
    uint32_t error_status;
    uint32_t error_status2;
    uint32_t error_mask_group[7];
    uint32_t error_group0[10];
    uint32_t error_group1[10];
    uint32_t error_group2[10];
    uint32_t error_group3[10];
    uint32_t error_group4[10];
    uint32_t error_group5[10];
    uint32_t error_group6[10];
    uint32_t error_group7[10];
    uint32_t warning_status;
    uint32_t encoder_reset;
    uint32_t encoder_type;
    uint32_t encoder_type2;
    uint32_t encoder_flags;
    uint32_t encoder_status;
    uint32_t phase_loss_detection;
    uint32_t group_touch[9];
    uint32_t device_node_group[2];
    uint32_t can_baudrate;
    uint32_t rs232_baudrate;
    uint32_t rs232_baudrate_debug[2];
    uint32_t rs485_baudrate;
    uint32_t rs485_baudrate_debug;
    uint32_t eeprom_group[33];
    uint32_t panel_group[36];
    uint32_t panel_group2[4];
    uint32_t device_time;
    uint32_t hardware_serial;
    uint32_t user_password;
    uint32_t factory_password;
    uint32_t device_restart;
    uint32_t test_group[27];
    uint32_t processor_group[29];
    uint32_t ecan_group[9];
    uint32_t interpolation_group[11];
    uint32_t parameter_checksum[4];
    uint32_t short_circuit_brake[9];
    uint32_t load_test_sweep[12];
    uint32_t quick_setting[7];
    uint32_t current_position_table;
    uint32_t encoder_communication_status;
    uint32_t group_resolver[10];
    uint32_t encoder_info;
    uint32_t encoder_warning;
    uint32_t encoder_temperature;
    uint32_t inertia_identification[38];
    uint32_t vibration_suppression_a[6];
    uint32_t low_frequency_vibration[11];
    uint32_t input_shaper[6];
    uint32_t model_tracking[3];
    uint32_t adaptive_notch_filter[16];
    uint32_t probe_function[13];
    uint32_t system_frequency_switch[3];
} motor_mfg_params_t;

/* PN相关参数结构体 */
typedef struct {
    uint32_t pn_software_version;
    uint32_t network_port_status;
    uint32_t pn_device_name;
    uint32_t pn_device_name1;
    uint32_t pn_device_name2;
    uint32_t pn_device_name3;
    uint32_t pn_device_name4;
    uint32_t ip_address;
    uint32_t subnet_mask;
    uint32_t default_gateway;
    uint32_t pn_enable;
    uint32_t software_limit_enable;
    uint32_t pn_user_rx_word;
    uint32_t pn_user_tx_word;
    uint32_t mac_address1;
    uint32_t mac_address2;
} motor_pn_params_t;

/* 电机参数区结构体 */
typedef struct {
    uint32_t motor_catalog_code;
    uint32_t motor_group[32];
    uint32_t support_mode;
    uint32_t manufacturer;
    uint32_t device_group[17];
} motor_param_area_t;

/* 电机参数结构体 */
typedef struct {
    /* 基本参数 */
    motor_basic_params_t basic;
    
    /* PID参数 */
    motor_pid_params_t pid;
    
    /* 校准参数 */
    motor_calib_params_t calib;
    
    /* 监控参数 */
    motor_monitor_params_t monitor;
    
    /* FOC控制参数 */
    motor_foc_params_t foc;
    
    /* 保护参数 */
    motor_protection_params_t protection;
    
    /* CANopen参数 */
    motor_canopen_params_t canopen;
    
    /* 通信参数 */
    motor_comm_params_t comm;
    
    /* 制造商特定参数 */
    motor_mfg_params_t mfg;
    
    /* PN相关参数 */
    motor_pn_params_t pn;
    
    /* DS402参数 */
    motor_ds402_params_t ds402;
    
    /* 电机参数区 */
    motor_param_area_t param_area;
    
    /* 电机状态 */
    motor_state_t state;
    
    /* 节点ID */
    uint32_t node_id;
    
    /* 初始化标志 */
    uint8_t initialized;
} motor_params_t;

/* 函数声明 */
void motor_params_init(motor_params_t *params);
void motor_params_load_default(motor_params_t *params);
void motor_params_update(motor_params_t *params, uint16_t param_id, float value);
float motor_params_get(motor_params_t *params, uint16_t param_id);
void motor_params_update_sub(motor_params_t *params, uint16_t param_id, uint8_t sub_index, uint32_t value);
uint32_t motor_params_get_sub(motor_params_t *params, uint16_t param_id, uint8_t sub_index);
void motor_params_clear_errors(motor_params_t *params);

/* 参数ID定义 */
enum {
    /* === 通信对象区 (0x1000 - 0x1FFF) === */
    /* 设备信息 */
    PARAM_ID_DEVICE_TYPE = 0x1000,          /* 设备类型 */
    PARAM_ID_ERROR_REGISTER = 0x1001,        /* 错误寄存器 */
    PARAM_ID_ERROR_FIELD = 0x1003,           /* 预定义错误字段 */
    PARAM_ID_SYNC_COB_ID = 0x1005,           /* 同步COB ID */
    PARAM_ID_SYNC_PERIOD = 0x1006,           /* 同步周期 */
    PARAM_ID_DEVICE_NAME = 0x1008,           /* 制造商设备名称 */
    PARAM_ID_HARDWARE_VERSION = 0x1009,       /* 制造商硬件版本 */
    PARAM_ID_SOFTWARE_VERSION = 0x100A,       /* 制造商软件版本 */
    PARAM_ID_NODE_ID = 0x100B,               /* 设备站号 */
    PARAM_ID_WATCHDOG_TIME = 0x100C,          /* 看门狗时间 */
    PARAM_ID_WATCHDOG_FACTOR = 0x100D,        /* 看门狗时间系数 */
    PARAM_ID_GUARD_COB_ID = 0x100E,           /* 节点保护ID */
    PARAM_ID_STORE_PARAMS = 0x1010,           /* 存储参数 */
    PARAM_ID_RESTORE_PARAMS = 0x1011,         /* 恢复默认参数 */
    PARAM_ID_EMERGENCY_COB_ID = 0x1014,       /* 紧急报文COB ID */
    PARAM_ID_HEARTBEAT_CONSUMER = 0x1016,      /* 心跳报文消费者 */
    PARAM_ID_HEARTBEAT_PRODUCER = 0x1017,      /* 心跳生产者时间 */
    PARAM_ID_IDENTITY_OBJECT = 0x1018,        /* 标识对象 */
    
    /* RPDO通信参数 */
    PARAM_ID_RPDO1_COMM = 0x1400,            /* RPDO1通信参数 */
    PARAM_ID_RPDO2_COMM = 0x1401,            /* RPDO2通信参数 */
    PARAM_ID_RPDO3_COMM = 0x1402,            /* RPDO3通信参数 */
    PARAM_ID_RPDO4_COMM = 0x1403,            /* RPDO4通信参数 */
    PARAM_ID_RPDO5_COMM = 0x1404,            /* RPDO5通信参数 */
    PARAM_ID_RPDO6_COMM = 0x1405,            /* RPDO6通信参数 */
    PARAM_ID_RPDO7_COMM = 0x1406,            /* RPDO7通信参数 */
    PARAM_ID_RPDO8_COMM = 0x1407,            /* RPDO8通信参数 */
    
    /* RPDO映射参数 */
    PARAM_ID_RPDO1_MAP = 0x1600,             /* RPDO1映射参数 */
    PARAM_ID_RPDO2_MAP = 0x1601,             /* RPDO2映射参数 */
    PARAM_ID_RPDO3_MAP = 0x1602,             /* RPDO3映射参数 */
    PARAM_ID_RPDO4_MAP = 0x1603,             /* RPDO4映射参数 */
    PARAM_ID_RPDO5_MAP = 0x1604,             /* RPDO5映射参数 */
    PARAM_ID_RPDO6_MAP = 0x1605,             /* RPDO6映射参数 */
    PARAM_ID_RPDO7_MAP = 0x1606,             /* RPDO7映射参数 */
    PARAM_ID_RPDO8_MAP = 0x1607,             /* RPDO8映射参数 */
    
    /* TPDO通信参数 */
    PARAM_ID_TPDO1_COMM = 0x1800,            /* TPDO1通信参数 */
    PARAM_ID_TPDO2_COMM = 0x1801,            /* TPDO2通信参数 */
    PARAM_ID_TPDO3_COMM = 0x1802,            /* TPDO3通信参数 */
    PARAM_ID_TPDO4_COMM = 0x1803,            /* TPDO4通信参数 */
    PARAM_ID_TPDO5_COMM = 0x1804,            /* TPDO5通信参数 */
    PARAM_ID_TPDO6_COMM = 0x1805,            /* TPDO6通信参数 */
    PARAM_ID_TPDO7_COMM = 0x1806,            /* TPDO7通信参数 */
    PARAM_ID_TPDO8_COMM = 0x1807,            /* TPDO8通信参数 */
    
    /* TPDO映射参数 */
    PARAM_ID_TPDO1_MAP = 0x1A00,             /* TPDO1映射参数 */
    PARAM_ID_TPDO2_MAP = 0x1A01,             /* TPDO2映射参数 */
    PARAM_ID_TPDO3_MAP = 0x1A02,             /* TPDO3映射参数 */
    PARAM_ID_TPDO4_MAP = 0x1A03,             /* TPDO4映射参数 */
    PARAM_ID_TPDO5_MAP = 0x1A04,             /* TPDO5映射参数 */
    PARAM_ID_TPDO6_MAP = 0x1A05,             /* TPDO6映射参数 */
    PARAM_ID_TPDO7_MAP = 0x1A06,             /* TPDO7映射参数 */
    PARAM_ID_TPDO8_MAP = 0x1A07,             /* TPDO8映射参数 */
    
    /* NMT启动/停止 */
    PARAM_ID_NMT_START_STOP = 0x1F80,         /* NMT启动/停止 */
    PARAM_ID_NMT_START_STOP_CH0 = 0x1F81,     /* NMT启动/停止 通道0 */
    PARAM_ID_NMT_START_STOP_CH1 = 0x1F82,     /* NMT启动/停止 通道1 */
    PARAM_ID_NMT_START_STOP_CH2 = 0x1F83,     /* NMT启动/停止 通道2 */
    PARAM_ID_NMT_START_STOP_CH3 = 0x1F84,     /* NMT启动/停止 通道3 */
    
    /* 同步输入输出 */
    PARAM_ID_SYNC_OUTPUT = 0x1FA0,            /* 同步输出 */
    PARAM_ID_SYNC_INPUT = 0x1FA1,             /* 同步输入 */
    
    /* === 制造商特定对象区 (0x2000 - 0x5FFF) === */
    PARAM_ID_MFG_CUSTOM_START = 0x2000,      /* 制造商特定区起始 */
    PARAM_ID_POWER_ON_ENABLE = 0x2000,        /* 上电自使能 */
    PARAM_ID_IO_GROUP = 0x2010,              /* IO组 */
    PARAM_ID_INPUT_FILTER = 0x2011,           /* 输入滤波组 */
    PARAM_ID_FAST_CAPTURE = 0x2012,           /* 快速捕捉模式 */
    PARAM_ID_RISING_CAPTURE_COUNT = 0x2013,   /* 上升沿捕捉个数 */
    PARAM_ID_RISING_CAPTURE_DATA = 0x2014,    /* 上升沿捕捉数据组 */
    PARAM_ID_FALLING_CAPTURE_COUNT = 0x2015,  /* 下降沿捕捉个数 */
    PARAM_ID_FALLING_CAPTURE_DATA = 0x2016,   /* 下降沿捕捉数据组 */
    PARAM_ID_INPUT_DELAY = 0x2017,            /* 输入延迟时间设置组 */
    PARAM_ID_CAPTURE_COMPENSATION = 0x2018,   /* 快速捕捉时间补偿 */
    PARAM_ID_DIN_CONTROL = 0x2020,            /* Din控制功能组 */
    PARAM_ID_INDEX_WINDOW = 0x2040,           /* 索引信号窗口 */
    PARAM_ID_OSCILLOSCOPE = 0x22F0,           /* 示波器组 */
    PARAM_ID_AUTO_FLIP = 0x2300,             /* 自动翻转组 */
    PARAM_ID_PHASE_CORRECTION = 0x2310,       /* 电机相位校正组 */
    PARAM_ID_SYSTEM_TEST = 0x2320,           /* 系统测试组 */
    PARAM_ID_SPECIAL_FUNCTION = 0x2330,       /* 特殊功能组 */
    PARAM_ID_HIGH_VOLTAGE_SAMPLING = 0x2401,  /* 高压速度采样时间 */
    PARAM_ID_ADC_GROUP = 0x2500,              /* 模数转换组 */
    PARAM_ID_ANALOG_INPUT = 0x2501,           /* 模拟输入功能组 */
    PARAM_ID_PULSE_GROUP2 = 0x2510,          /* 脉冲组2 */
    PARAM_ID_PULSE_GROUP = 0x2520,           /* 脉冲组 */
    PARAM_ID_PULSE_GROUP2B = 0x2521,         /* 脉冲组2 */
    PARAM_ID_FULL_CLOSED_LOOP = 0x2530,       /* 全闭环组 */
    PARAM_ID_NOTCH_FILTER = 0x2540,           /* 陷波滤波器组 */
    PARAM_ID_NOTCH_FILTER2 = 0x2541,          /* 陷波滤波器组 */
    PARAM_ID_FIELD_WEAKENING = 0x2550,        /* 弱磁控制参数组 */
    PARAM_ID_FILTER_EXTENSION = 0x2560,       /* 滤波器扩展参数组 */
    PARAM_ID_ONLINE_TUNING = 0x2600,          /* 在线整定组 */
    PARAM_ID_ERROR_MASK = 0x2F00,            /* 错误掩码 */
    PARAM_ID_ERROR_STATUS = 0x2F10,          /* 错误状态 */
    PARAM_ID_ERROR_STATUS2 = 0x2F20,         /* 错误状态2 */
    PARAM_ID_ERROR_MASK_GROUP = 0x2F40,       /* 错误掩码组 */
    PARAM_ID_ERROR_GROUP0 = 0x2F80,          /* 错误组0 */
    PARAM_ID_ERROR_GROUP1 = 0x2F81,          /* 错误组1 */
    PARAM_ID_ERROR_GROUP2 = 0x2F82,          /* 错误组2 */
    PARAM_ID_ERROR_GROUP3 = 0x2F83,          /* 错误组3 */
    PARAM_ID_ERROR_GROUP4 = 0x2F84,          /* 错误组4 */
    PARAM_ID_ERROR_GROUP5 = 0x2F85,          /* 错误组5 */
    PARAM_ID_ERROR_GROUP6 = 0x2F86,          /* 错误组6 */
    PARAM_ID_ERROR_GROUP7 = 0x2F87,          /* 错误组7 */

    PARAM_ID_WARNING_STATUS = 0x3000,        /* 警告状态字 */
    PARAM_ID_ENCODER_RESET = 0x3010,         /* 通讯编码器数据复位 */
    PARAM_ID_ENCODER_TYPE = 0x3014,          /* 编码器类型 */
    PARAM_ID_ENCODER_TYPE2 = 0x3015,         /* 编码器类型2 */
    PARAM_ID_ENCODER_FLAGS = 0x3016,         /* 编码器标志位 */
    PARAM_ID_ENCODER_STATUS = 0x3017,        /* 编码器状态 */
    PARAM_ID_PHASE_LOSS_DETECTION = 0x3040,   /* 电源输入缺相检测控制 */
    PARAM_ID_GROUP_TOUCH = 0x3E00,           /* Group_Touch */
    PARAM_ID_DEVICE_NODE_GROUP = 0x3F00,      /* 设备站号组 */
    PARAM_ID_CAN_BAUDRATE = 0x3F10,          /* CAN波特率 */
    PARAM_ID_RS232_BAUDRATE = 0x3F20,        /* RS232波特率 */
    PARAM_ID_RS232_BAUDRATE_DEBUG = 0x3F21,  /* RS232波特率（调试用） */
    PARAM_ID_RS485_BAUDRATE = 0x3F30,        /* RS485波特率 */
    PARAM_ID_RS485_BAUDRATE_DEBUG = 0x3F31,  /* RS485波特率（调试用） */
    PARAM_ID_EEPROM_GROUP = 0x3F40,          /* Eeprom组 */
    PARAM_ID_PANEL_GROUP = 0x3F80,           /* 面板组 */
    PARAM_ID_PANEL_GROUP2 = 0x3F81,          /* 面板组2 */

    PARAM_ID_DEVICE_TIME = 0x4000,           /* 设备时间 */
    PARAM_ID_HARDWARE_SERIAL = 0x4010,       /* 硬件序列号 */
    PARAM_ID_USER_PASSWORD = 0x4020,         /* 用户密码 */
    PARAM_ID_FACTORY_PASSWORD = 0x4030,      /* 厂家密码 */
    PARAM_ID_DEVICE_RESTART = 0x4040,        /* 设备重启动 */
    PARAM_ID_TEST_GROUP = 0x4080,            /* 测试组 */
    PARAM_ID_PROCESSOR_GROUP = 0x4090,       /* 处理器组 */
    PARAM_ID_ECAN_GROUP = 0x40A0,            /* ECAN组 */
    PARAM_ID_INTERPOLATION_GROUP = 0x40B0,   /* 插补组 */
    PARAM_ID_PARAMETER_CHECKSUM = 0x40C0,    /* 参数校验组 */
    PARAM_ID_SHORT_CIRCUIT_BRAKE = 0x40D0,   /* 短路制动组 */
    PARAM_ID_LOAD_TEST_SWEEP = 0x40E0,       /* 负载测定与扫频组 */
    PARAM_ID_QUICK_SETTING = 0x40F0,         /* 快速设定组 */
    PARAM_ID_CURRENT_POSITION_TABLE = 0x40F8, /* 当前位置表索引 */
    PARAM_ID_ENCODER_COMM_STATUS = 0x40F9,   /* 编码器通信状态 */
    PARAM_ID_GROUP_RESOLVER = 0x4100,        /* Group_Resolver */
    PARAM_ID_ENCODER_INFO = 0x4101,          /* 编码器信息 */
    PARAM_ID_ENCODER_WARNING = 0x4102,       /* 编码器警告信息 */
    PARAM_ID_ENCODER_TEMPERATURE = 0x4103,   /* 编码器温度 */
    PARAM_ID_INERTIA_IDENTIFICATION = 0x4200, /* 惯量辨识及自整定 */
    PARAM_ID_VIBRATION_SUPPRESSION_A = 0x4300, /* A型振动抑制控制组 */
    PARAM_ID_LOW_FREQUENCY_VIBRATION = 0x4310, /* 低频振动抑制组 */
    PARAM_ID_INPUT_SHAPER = 0x4320,          /* 输入整形器组 */
    PARAM_ID_MODEL_TRACKING = 0x4330,        /* 模型跟踪控制组 */
    PARAM_ID_ADAPTIVE_NOTCH_FILTER = 0x4400, /* 自适应陷波滤波器组 */
    PARAM_ID_PROBE_FUNCTION = 0x4500,        /* 探针功能对象组 */
    PARAM_ID_SYSTEM_FREQUENCY_SWITCH = 0x4600, /* 系统频率切换参数对象组 */
    PARAM_ID_PN_SOFTWARE_VERSION = 0x4700,   /* PN软件版本 */
    PARAM_ID_NETWORK_PORT_STATUS = 0x4702,   /* 网口状态 */
    PARAM_ID_PN_DEVICE_NAME = 0x4703,        /* PN设备名称 */
    PARAM_ID_PN_DEVICE_NAME1 = 0x4704,       /* PN设备名称1 */
    PARAM_ID_PN_DEVICE_NAME2 = 0x4705,       /* PN设备名称2 */
    PARAM_ID_PN_DEVICE_NAME3 = 0x4706,       /* PN设备名称3 */
    PARAM_ID_PN_DEVICE_NAME4 = 0x4707,       /* PN设备名称4 */
    PARAM_ID_IP_ADDRESS = 0x4708,            /* IP地址 */
    PARAM_ID_SUBNET_MASK = 0x4709,           /* 子网掩码 */
    PARAM_ID_DEFAULT_GATEWAY = 0x470A,       /* 默认网关 */
    PARAM_ID_PN_ENABLE = 0x470B,             /* PN使能 */
    PARAM_ID_SOFTWARE_LIMIT_ENABLE = 0x470C, /* 软件限位使能 */
    PARAM_ID_PN_USER_RX_WORD = 0x470D,       /* PN用户自定义接收字 */
    PARAM_ID_PN_USER_TX_WORD = 0x470E,       /* PN用户自定义发送字 */
    PARAM_ID_MAC_ADDRESS1 = 0x470F,          /* Mac地址1 */
    PARAM_ID_MAC_ADDRESS2 = 0x4710,          /* Mac地址2 */
    
    PARAM_ID_MFG_ID = 0x5000,               /* 制造商ID */
    PARAM_ID_MFG_VERSION = 0x5001,           /* 制造商版本信息 */
    PARAM_ID_MFG_SERIAL = 0x5002,            /* 制造商序列号 */
    PARAM_ID_MFG_CUSTOM_END = 0x5FFF,        /* 制造商特定区结束 */
    
    /* === DS402 驱动配置对象区 (0x6000 - 0x6FFF) === */
    PARAM_ID_MOTOR_ABSOLUTE_POSITION = 0x6000, /* 电机绝对位置 */
    PARAM_ID_COMMUNICATION_INTERRUPT_MODE = 0x6010, /* 通讯中断模式 */
    PARAM_ID_ERROR_CODE = 0x6030,            /* 错误代码 */
    PARAM_ID_DS402_CONTROL_WORD = 0x6040,    /* 控制字 */
    PARAM_ID_DS402_STATUS_WORD = 0x6041,     /* 状态字 */
    PARAM_ID_QUICK_STOP_MODE = 0x6050,       /* 快速停止模式 */
    PARAM_ID_SHUTDOWN_STOP_MODE = 0x6051,    /* 关机停止模式 */
    PARAM_ID_DISABLE_STOP_MODE = 0x6052,     /* 禁止停止模式 */
    PARAM_ID_HALT_MODE = 0x6053,             /* 暂停模式 */
    PARAM_ID_ERROR_STOP_MODE = 0x6054,        /* 报错停止模式 */
    PARAM_ID_DS402_OPERATION_MODE = 0x6060,  /* 操作模式 */
    PARAM_ID_ACTIVE_OPERATION_MODE = 0x6061,  /* 有效工作模式 */
    PARAM_ID_EFFECTIVE_TARGET_POSITION = 0x6062, /* 有效目标位置(齿轮计算) */
    PARAM_ID_ACTUAL_POSITION_INC = 0x6063,    /* 实际位置-inc */
    PARAM_ID_DS402_ACTUAL_POSITION = 0x6064, /* 实际位置 */
    PARAM_ID_ACTUAL_POSITION_PULSE = 0x606F,   /* 实际位置脉冲 */
    PARAM_ID_MAX_FOLLOWING_ERROR = 0x6065,    /* 最大跟随误差 */
    PARAM_ID_TARGET_POSITION_WINDOW = 0x6067,  /* 目标位置窗口 */
    PARAM_ID_POSITION_WINDOW_TIME = 0x6068,    /* 位置窗口时间 */
    PARAM_ID_EFFECTIVE_TARGET_VELOCITY = 0x6069, /* 有效目标速度 */
    PARAM_ID_ACTUAL_VELOCITY = 0x606B,        /* 实际速度 */
    PARAM_ID_DS402_ACTUAL_VELOCITY = 0x606C, /* 实际速度 */
    PARAM_ID_TARGET_TORQUE_PERCENT = 0x6071,   /* 目标扭矩% */
    PARAM_ID_MAX_TORQUE_LIMIT_PERCENT = 0x6072, /* 最大扭矩限制% */
    PARAM_ID_TARGET_CURRENT_LIMIT = 0x6073,     /* 目标电流限制 */
    PARAM_ID_MOTOR_RATED_CURRENT = 0x6075,      /* 电机额定电流 */
    PARAM_ID_MOTOR_RATED_TORQUE = 0x6076,       /* 电机额定扭矩 */
    PARAM_ID_DS402_ACTUAL_TORQUE = 0x6077,   /* 实际转矩 */
    PARAM_ID_ACTUAL_TORQUE = 0x6077,            /* 实际转矩 */
    PARAM_ID_ACTUAL_CURRENT = 0x6078,           /* 实际电流 */
    PARAM_ID_ACTUAL_BUS_VOLTAGE_MV = 0x6079,    /* 实际总线电压_mV */
    PARAM_ID_DS402_TARGET_POSITION = 0x607A, /* 目标位置 */
    PARAM_ID_HOME_OFFSET = 0x607C,              /* 原点偏移 */
    PARAM_ID_SOFT_LIMIT = 0x607D,               /* 软限位组 */
    PARAM_ID_VELOCITY_POSITION_DIRECTION = 0x607E, /* 速度位置方向控制 */
    PARAM_ID_MAX_VELOCITY_LIMIT = 0x6080,       /* 最大速度限制 */
    PARAM_ID_MAX_VELOCITY_LIMIT_RPM = 0x6081,   /* 最大速度限制rpm */
    PARAM_ID_TRAPEZOIDAL_VELOCITY = 0x6082,     /* 梯形速度 */
    PARAM_ID_TRAPEZOIDAL_ACCELERATION = 0x6083, /* 梯形加速度 */
    PARAM_ID_TRAPEZOIDAL_DECELERATION = 0x6084, /* 梯形减速度 */
    PARAM_ID_QUICK_STOP_DECELERATION = 0x6085,  /* 快速停止减速度 */
    PARAM_ID_POSITION_ENCODER_RESOLUTION = 0x6090, /* 位置编码器分辨率 */
    PARAM_ID_DS402_ENCODER_RESOLUTION = 0x6091,       /* 编码器分辨率 */
    PARAM_ID_MOTOR_REVOLUTIONS = 0x6092,        /* 电机旋转数 */
    PARAM_ID_BUS_GEAR_RATIO = 0x6093,           /* 总线电子齿轮比 */
    PARAM_ID_FEED_CONSTANT = 0x6094,            /* Feed constant */
    PARAM_ID_POS_FACTOR = 0x6095,               /* Group_Pos_Factor */
    PARAM_ID_SPEED_FACTOR = 0x6096,             /* Group_Speed_Factor */
    PARAM_ID_ACC_FACTOR = 0x6097,               /* Group_Acc_Factor */
    PARAM_ID_HOME_MODE = 0x6098,                /* 原点模式 */
    PARAM_ID_HOME_SPEED = 0x6099,               /* 原点速度组 */
    PARAM_ID_HOME_ACCELERATION = 0x609A,        /* 原点加速度 */
    PARAM_ID_CAPTURE_FUNCTION = 0x60B0,         /* 捕捉功能 */
    PARAM_ID_CAPTURE_STATUS = 0x60B1,           /* 捕捉状态 */
    PARAM_ID_RISING_EDGE_CAPTURE_POSITION1 = 0x60B2, /* 上升沿捕捉位置1 */
    PARAM_ID_FALLING_EDGE_CAPTURE_POSITION1 = 0x60B3, /* 下降沿捕捉位置1 */
    PARAM_ID_RISING_EDGE_CAPTURE_POSITION2 = 0x60B4, /* 上升沿捕捉位置2 */
    PARAM_ID_FALLING_EDGE_CAPTURE_POSITION2 = 0x60B5, /* 下降沿捕捉位置2 */
    PARAM_ID_INTERPOLATION_SUBMODE = 0x60C0,     /* 插补子模式 */
    PARAM_ID_INTERPOLATION_DATA = 0x60C1,        /* 插补数据组 */
    PARAM_ID_INTERPOLATION_TIME = 0x60C2,        /* 插补时间组 */
    PARAM_ID_IP_SYNC_DEFINITION = 0x60C3,        /* Ip_Sync_Definition */
    PARAM_ID_GROUP_IP_DATA_CFG = 0x60C4,         /* Group_Ip_Data_CFG */
    PARAM_ID_POSITION_FOLLOWING_ERROR = 0x60F4,  /* 位置跟随误差 */
    PARAM_ID_CURRENT_LOOP_GROUP2 = 0x60F6,       /* 电流环组2 */
    PARAM_ID_CURRENT_GROUP = 0x60F8,            /* 电流组 */
    PARAM_ID_POWER_GROUP = 0x60F9,              /* 功率组 */
    PARAM_ID_SPEED_GROUP = 0x60FA,              /* 速度组 */
    PARAM_ID_POSITION_GROUP = 0x60FB,           /* 位置组 */
    PARAM_ID_EFFECTIVE_TARGET_POSITION2 = 0x60FC, /* 有效目标位置 */
    PARAM_ID_INPUT_STATUS = 0x60FD,            /* 输入状态 */
    PARAM_ID_OUTPUT_GROUP = 0x60FE,            /* 输出组 */
    PARAM_ID_TARGET_VELOCITY = 0x60FF,         /* 目标速度 */
    
    /* === 电机参数区 (0x6400 - 0x6FFF) === */
    /* 电机目录代码 */
    PARAM_ID_MOTOR_CATALOG_CODE = 0x6400,      /* 电机目录代码 */
    
    /* 基本参数 */
    PARAM_ID_POLE_PAIRS = 0x6401,            /* 极对数 */
    PARAM_ID_BUS_VOLTAGE = 0x6402,           /* 母线电压 */
    PARAM_ID_MAX_CURRENT = 0x6403,           /* 最大电流 */
    PARAM_ID_MAX_VELOCITY = 0x6404,          /* 最大速度 */
    PARAM_ID_MAX_TORQUE = 0x6405,            /* 最大转矩 */
    PARAM_ID_CONTROL_DT = 0x6406,            /* 控制周期 */
    PARAM_ID_PWM_FREQUENCY = 0x6407,         /* PWM频率 */
    
    /* 传感器参数 */
    PARAM_ID_ENCODER_RESOLUTION = 0x6410,    /* 编码器分辨率 */
    
    /* 校准参数 */
    PARAM_ID_PHASE_OFFSET = 0x6430,          /* 相位偏移 */
    PARAM_ID_CURRENT_OFFSET_A = 0x6431,      /* A相电流偏移 */
    PARAM_ID_CURRENT_OFFSET_B = 0x6432,      /* B相电流偏移 */
    
    /* 温度参数 */
    PARAM_ID_TEMP_THRESHOLD = 0x6440,        /* 温度阈值 */
    PARAM_ID_TEMP_HYSTERESIS = 0x6441,       /* 温度回差 */
    
    /* 监控参数 */
    PARAM_ID_U_PHASE_CURRENT = 0x6450,       /* U相电流 */
    PARAM_ID_V_PHASE_CURRENT = 0x6451,       /* V相电流 */
    PARAM_ID_W_PHASE_CURRENT = 0x6452,       /* W相电流 */
    PARAM_ID_BUS_VOLTAGE_MON = 0x6453,       /* 母线电压 */
    PARAM_ID_TEMPERATURE_MON = 0x6454,       /* 温度 */
    
    /* FOC控制参数 */
    PARAM_ID_FOC_MAX_PWM_VALUE = 0x6460,     /* FOC最大PWM值 */
    PARAM_ID_FOC_SAMPLE_FREQ = 0x6461,       /* FOC采样频率 */
    PARAM_ID_CONTROL_MODE = 0x6462,          /* 控制模式 */
    PARAM_ID_TARGET_POSITION = 0x6463,       /* 目标位置 */
    PARAM_ID_TARGET_VELOCITY_MOTOR = 0x6464,       /* 目标速度 */
    PARAM_ID_TARGET_TORQUE = 0x6465,         /* 目标转矩 */
    PARAM_ID_CURRENT_POSITION = 0x6466,      /* 当前位置 */
    PARAM_ID_CURRENT_VELOCITY = 0x6467,      /* 当前速度 */
    PARAM_ID_CURRENT_TORQUE = 0x6468,        /* 当前转矩 */
    PARAM_ID_POSITION_KP = 0x6469,           /* 位置环KP */
    PARAM_ID_POSITION_KI = 0x646A,           /* 位置环KI */
    PARAM_ID_POSITION_KD = 0x646B,           /* 位置环KD */
    PARAM_ID_VELOCITY_KP = 0x646C,           /* 速度环KP */
    PARAM_ID_VELOCITY_KI = 0x646D,           /* 速度环KI */
    PARAM_ID_VELOCITY_KD = 0x646E,           /* 速度环KD */
    PARAM_ID_CURRENT_KP = 0x646F,            /* 电流环KP */
    PARAM_ID_CURRENT_KI = 0x6470,            /* 电流环KI */
    PARAM_ID_CURRENT_KD = 0x6471,            /* 电流环KD */
    PARAM_ID_FOC_ENABLED = 0x6472,           /* FOC使能状态 */
    
    /* FOC角度结构体参数 */
    PARAM_ID_ANGLE_ANGLE = 0x6473,           /* 电角度 */
    PARAM_ID_ANGLE_SIN = 0x6474,             /* 角度正弦值 */
    PARAM_ID_ANGLE_COS = 0x6475,             /* 角度余弦值 */
    
    /* FOC三相电流结构体参数 */
    PARAM_ID_CURRENT_UVW_U = 0x6476,         /* U相电流 */
    PARAM_ID_CURRENT_UVW_V = 0x6477,         /* V相电流 */
    PARAM_ID_CURRENT_UVW_W = 0x6478,         /* W相电流 */
    
    /* FOC Alpha-Beta电流结构体参数 */
    PARAM_ID_CURRENT_ALPHA = 0x6479,         /* Alpha轴电流 */
    PARAM_ID_CURRENT_BETA = 0x647A,          /* Beta轴电流 */
    
    /* FOC DQ电流结构体参数 */
    PARAM_ID_CURRENT_D = 0x647B,             /* D轴电流 */
    PARAM_ID_CURRENT_Q = 0x647C,             /* Q轴电流 */
    
    /* FOC DQ电压结构体参数 */
    PARAM_ID_VOLTAGE_D = 0x647D,             /* D轴电压 */
    PARAM_ID_VOLTAGE_Q = 0x647E,             /* Q轴电压 */
    
    /* FOC PWM结构体参数 */
    PARAM_ID_PWM_U = 0x647F,                /* U相PWM */
    PARAM_ID_PWM_V = 0x6480,                /* V相PWM */
    PARAM_ID_PWM_W = 0x6481,                /* W相PWM */
    
    /* 保护参数 */
    PARAM_ID_MAX_CURRENT_PROT = 0x6482,      /* 最大电流保护 */
    PARAM_ID_MIN_VOLTAGE_PROT = 0x6483,      /* 最小电压保护 */
    PARAM_ID_MAX_VOLTAGE_PROT = 0x6484,      /* 最大电压保护 */
    PARAM_ID_MAX_TEMPERATURE_PROT = 0x6485,  /* 最大温度保护 */
    PARAM_ID_OVERCURRENT_PROT = 0x6486,      /* 过流保护使能 */
    PARAM_ID_UNDERVOLT_PROT = 0x6487,        /* 欠压保护使能 */
    PARAM_ID_OVERVOLT_PROT = 0x6488,         /* 过压保护使能 */
    PARAM_ID_OVERTEMPERATURE_PROT = 0x6489,  /* 过热保护使能 */
    PARAM_ID_FAULT_STATUS = 0x648A,          /* 故障状态 */
    PARAM_ID_FAULT_ACTIVE = 0x648B,          /* 故障激活 */
    
    /* 采样频率参数 */
    PARAM_ID_MONITOR_SAMPLE_FREQ = 0x648C,   /* 监控采样频率 */
    
    /* 无感控制参数 */
    PARAM_ID_SENSORLESS_ENABLE = 0x6490,      /* 无感控制使能 */
    PARAM_ID_SENSORLESS_K1 = 0x6491,          /* 无感控制参数K1 */
    PARAM_ID_SENSORLESS_K2 = 0x6492,          /* 无感控制参数K2 */
    PARAM_ID_SENSORLESS_K3 = 0x6493,          /* 无感控制参数K3 */
    PARAM_ID_SENSORLESS_MIN_SPEED = 0x6494,   /* 无感控制最小速度 */
    PARAM_ID_SENSORLESS_MAX_SPEED = 0x6495,   /* 无感控制最大速度 */
    PARAM_ID_SENSORLESS_ESTIMATOR_GAIN = 0x6496, /* 观测器增益 */
    PARAM_ID_ESTIMATED_SPEED = 0x6497,       /* 估算速度 */
    PARAM_ID_ESTIMATED_ANGLE = 0x6498,       /* 估算角度 */
    
    /* 无感控制反电动势估计结构体参数 */
    PARAM_ID_BACK_EMF_D = 0x6499,           /* D轴反电动势 */
    PARAM_ID_BACK_EMF_Q = 0x649A,           /* Q轴反电动势 */
    
    /* 电机组 */
    PARAM_ID_MOTOR_GROUP = 0x6500,            /* 电机组 */
    /* 支持模式 */
    PARAM_ID_SUPPORT_MODE = 0x6600,           /* 支持模式 */
    /* 制造商 */
    PARAM_ID_MANUFACTURER = 0x6700,           /* 制造商 */
    /* 设备组 */
    PARAM_ID_DEVICE_GROUP = 0x6800,           /* 设备组 */
    
    PARAM_ID_MAX
};

#endif /* MOTOR_PARAMS_H */