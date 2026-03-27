#include "joint_params.h"

const ParamItemTypeDef ParamTable[PARAM_TOTAL_COUNT] = {

/*********** P0 监控参数 2000~2048 (49个 只读) ***********/
{0x2000, "电机转速",          PARAM_READ_ONLY,  PARAM_TYPE_I16, -32768, 32767, "Rpm"},
{0x2001, "电机负载率",        PARAM_READ_ONLY,  PARAM_TYPE_I16, -400,   400,   "%"},
{0x2002, "电角度",            PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      359,   "°"},
{0x2003, "DI输入电平",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      255,   "/"},
{0x2004, "DO输出电平",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      15,    "/"},
{0x2002, "电角度",            PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      359,   "°"},
{0x2003, "DI输入电平",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      255,   "/"},
{0x2004, "DO输出电平",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      15,    "/"},
{0x2005, "电机端编码器多圈值",  PARAM_READ_ONLY,  PARAM_TYPE_I16, -32768, 32767, "/"},
{0x2006, "系统上电时间",       PARAM_READ_ONLY,  PARAM_TYPE_U32, 0,      0xFFFFFFFF, "Min"},
{0x2007, "MCU温度",           PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "℃"},
{0x2008, "电机温度",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "℃"},
{0x2009, "编码器扇区",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      5,     "/"},
{0x2010, "母线电压",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      1000,  "V"},
{0x2011, "电机电流",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      655,   "A"},
{0x2012, "驱动器状态",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      10,    "/"},
{0x2013, "制动负载率",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      400,   "%"},
{0x2014, "MOS管温度",         PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      200,   "℃"},
{0x2015, "脉冲指令",          PARAM_READ_ONLY,  PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2016, "故障码",            PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      255,   "/"},
{0x2017, "故障转速",          PARAM_READ_ONLY,  PARAM_TYPE_I16, -32767, 32767, "/"},
{0x2018, "故障母线电压",      PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      1000,  "V"},
{0x2019, "故障电流有效值",    PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      655,   "A"},
{0x2020, "故障时温度",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "℃"},    
{0x2021, "编码器单圈值",      PARAM_READ_ONLY,  PARAM_TYPE_U32, 0,      0xFFFFFFFF,"Pulse"},
{0x2022, "负载惯量",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      655,   "kg.cm2"},
{0x2023, "负载惯量比",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      655,   "%"},
{0x2024, "反馈脉冲总数",      PARAM_READ_ONLY,  PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2025, "位置指令频率",      PARAM_READ_ONLY,  PARAM_TYPE_U32, 0,      0xFFFFFFFF,"kHz"},
{0x2026, "CPU软件版本",       PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2027, "EEPROM数据版本",    PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2028, "主编码器CRC错误计数",PARAM_READ_ONLY,PARAM_TYPE_U16, 0,      65535, "/"},
{0x2029, "主编码器无数据计数",PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2030, "主编码器内部故障计数",PARAM_READ_ONLY,PARAM_TYPE_U16, 0,      65535, "/"},
{0x2031, "第二编码器CRC错误计数",PARAM_READ_ONLY,PARAM_TYPE_U16,0,65535,"/"},
{0x2032, "第二编码器无数据错误计数",PARAM_READ_ONLY,PARAM_TYPE_U16,0,65535,"/"},
{0x2033, "第二编码器内部故障计数",PARAM_READ_ONLY,PARAM_TYPE_U16,0,65535,"/"},
{0x2034, "Ecat状态",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2035, "速度指令",          PARAM_READ_ONLY,  PARAM_TYPE_I16, -32768, 32767, "/"},
{0x2036, "第二编码器单圈值",  PARAM_READ_ONLY,  PARAM_TYPE_U32, 0,      0xFFFFFFFF,"/"},
{0x2037, "当前故障码",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2038, "Ecat故障码",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2039, "硬件版本",          PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2040, "电机额定电流",      PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      655,   "A"},
{0x2041, "私服驱动器代码",    PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2042, "输出端位置",        PARAM_READ_ONLY,  PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"/"},
{0x2043, "电机端位置",        PARAM_READ_ONLY,  PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"/"},
{0x2044, "伺服控制源",        PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},
{0x2045, "输出端多圈值",      PARAM_READ_ONLY,  PARAM_TYPE_I16, -32768, 32767, "/"},
{0x2046, "输出端传感器值",    PARAM_READ_ONLY,  PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"/"},
{0x2047, "输出端传感器值16",  PARAM_READ_ONLY,  PARAM_TYPE_I16, -32768, 32767, "/"},
{0x2048, "保留",              PARAM_READ_ONLY,  PARAM_TYPE_U16, 0,      65535, "/"},

/*********** P1 基本参数 2100~2162 (63个 读写) ***********/
{0x2100, "控制模式选择",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      8,     "/"},
{0x2101, "位置指令源选择",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2102, "外部脉冲指令输入形式",PARAM_READ_WRITE,PARAM_TYPE_U16, 0,      5,     "/"},
{0x2103, "内部多段位置指令执行模式",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4,"/"},
{0x2104, "电机方向",          PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2105, "速度指令源选择",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      5,     "/"},
{0x2106, "内部多段速度指令执行模式",PARAM_READ_WRITE,PARAM_TYPE_U16,0,2,"/"},
{0x2107, "分频输出",          PARAM_READ_WRITE, PARAM_TYPE_U16, 165,    16383, "Pulse"},
{0x2108, "转矩指令源选择",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x2109, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2110, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2111, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2112, "位置指令加速时间",  PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     10000, "ms"},
{0x2113, "位置指令减速时间",  PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     10000, "ms"},
{0x2114, "位置指令S平滑时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "ms"},
{0x2115, "速度指令加速时间",  PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     30000, "ms"},
{0x2116, "速度指令减速时间",  PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     30000, "ms"},
{0x2117, "速度指令S平滑时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2118, "零速停车时间",      PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     30000, "ms"},
{0x2119, "超程保护停车时间",  PARAM_READ_WRITE, PARAM_TYPE_U16, 50,     30000, "ms"},
{0x2120, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2121, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2122, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2123, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2124, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2125, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2126, "电子齿轮比分子L",   PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      65535, "/"},
{0x2127, "电子齿轮比分子H",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2128, "电子齿轮比分母L",   PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      65535, "/"},
{0x2129, "电子齿轮比分母H",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2130, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2131, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      65535, "/"},
{0x2132, "定位接近宽度",      PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      65535, "Pulse"},
{0x2133, "定位完成宽度",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "Pulse"},
{0x2134, "位置偏差清除DI信号选择",PARAM_READ_WRITE,PARAM_TYPE_U16,0,3,"/"},
{0x2135, "位置偏差自动清除选择",PARAM_READ_WRITE,PARAM_TYPE_U16, 0,      2,     "/"},
{0x2136, "位置跟随偏差报警阈值",PARAM_READ_WRITE,PARAM_TYPE_U16, 0,      65535, "/"},
{0x2137, "位置跟随偏差故障阈值",PARAM_READ_WRITE,PARAM_TYPE_U16, 0,      65535, "/"},
{0x2138, "转矩达到门限制",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "%"},
{0x2139, "最高转速设定",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      4000,  "Rpm"},
{0x2140, "零速信号输出值",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "Rpm"},
{0x2141, "旋转信号输出值",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "Rpm"},
{0x2142, "速度接近门限",      PARAM_READ_WRITE, PARAM_TYPE_U16, 10,     4000,  "Rpm"},
{0x2143, "速度达到门限制",    PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      4000,  "Rpm"},
{0x2144, "模拟速度指令零位固定值",PARAM_READ_WRITE,PARAM_TYPE_U16,0,300,"Rpm"},
{0x2145, "Z脉冲输出宽度",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "ms"},
{0x2146, "正转最大转矩限制",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      400,   "%"},
{0x2147, "反转最大转矩限制",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      400,   "%"},
{0x2148, "转矩限制来源选择",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x2149, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "Hz"},
{0x2150, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2151, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2152, "二级故障停机方式",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2153, "停机模式选择",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2154, "伺服ON接受指令延迟",PARAM_READ_WRITE,PARAM_TYPE_U16, 0,      500,   "ms"},
{0x2155, "制动器指令伺服off延迟",PARAM_READ_WRITE,PARAM_TYPE_U16,0,500,"ms"},
{0x2156, "制动指令输出速度值",PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      65535, "Rpm"},
{0x2157, "伺服OFF制动等待时间",PARAM_READ_WRITE,PARAM_TYPE_U16,1,1000,"ms"},
{0x2158, "分频输出脉冲方向选择",PARAM_READ_WRITE,PARAM_TYPE_U16,0,1,"/"},
{0x2159, "保留",              PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x2160, "故障显示选择",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x2161, "系统参数初始化",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2162, "厂家密码",          PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},

/*********** P2 多段位置 2200~2247 (48个 读写) ***********/
{0x2200, "内部位置指令1",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2201, "内置位置指令1移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2202, "等待时间1",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2203, "内部位置指令2",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2204, "内置位置指令2移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2205, "等待时间2",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2206, "内部位置指令3",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2207, "内置位置指令3移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2208, "等待时间3",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2209, "内部位置指令4",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2210, "内置位置指令4移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2211, "等待时间4",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2212, "内部位置指令5",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2213, "内置位置指令5移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2214, "等待时间5",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2215, "内部位置指令6",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2216, "内置位置指令6移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2217, "等待时间6",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2218, "内部位置指令7",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2219, "内置位置指令7移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2220, "等待时间7",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2221, "内部位置指令8",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2222, "内置位置指令8移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2223, "等待时间8",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2224, "内部位置指令9",     PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2225, "内置位置指令9移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2226, "等待时间9",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2227, "内部位置指令10",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2228, "内置位置指令10移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2229, "等待时间10",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2230, "内部位置指令11",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2231, "内置位置指令11移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2232, "等待时间11",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2233, "内部位置指令12",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2234, "内置位置指令12移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2235, "等待时间12",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2236, "内部位置指令13",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2237, "内置位置指令13移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2238, "等待时间13",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2239, "内部位置指令14",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2240, "内置位置指令14移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2241, "等待时间14",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2242, "内部位置指令15",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2243, "内置位置指令15移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2244, "等待时间15",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},
{0x2245, "内部位置指令16",    PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000000,0x7FFFFFFF,"Pulse"},
{0x2246, "内置位置指令16移动速度",PARAM_READ_WRITE,PARAM_TYPE_U16,0,4000,"Rpm"},
{0x2247, "等待时间16",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      30000, "ms"},

/*********** P3 多段速度 2300~2331 (32个 读写) ***********/
{0x2300, "第1段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2301, "第1段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2302, "第2段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2303, "第2段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2304, "第3段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2305, "第3段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2306, "第4段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2307, "第4段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2308, "第5段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2309, "第5段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2310, "第6段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2311, "第6段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2312, "第7段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2313, "第7段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},   
{0x2314, "第8段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2315, "第8段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2316, "第9段转速",        PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2317, "第9段运行时间",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2318, "第10段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2319, "第10段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2320, "第11段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2321, "第11段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2322, "第12段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2323, "第12段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2324, "第13段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2325, "第13段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2326, "第14段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2327, "第14段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2328, "第15段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2329, "第15段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},
{0x2330, "第16段转速",       PARAM_READ_WRITE, PARAM_TYPE_I16, -4000, 4000, "Rpm"},
{0x2331, "第16段运行时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "Sec"},

/*********** P4 转矩参数 2400~2401 (2个 读写) ***********/
{0x2400, "内部转矩指令",     PARAM_READ_WRITE, PARAM_TYPE_I16, -400,  400,  "%"},
{0x2401, "转速限制",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      4000,  "Rpm"},

/*********** P5 增益参数 2500~2531 (32个 读写) ***********/
{0x2500, "位置调节器比例增益",PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      2000, "Rad/s"},
{0x2501, "速度调节器比例增益KVP",PARAM_READ_WRITE,PARAM_TYPE_U16,1,5000,"/"},
{0x2502, "速度调节器积分时间常数",PARAM_READ_WRITE,PARAM_TYPE_U16,1,3000,"/"},
{0x2503, "增益调整模式",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      2,     "/"},
{0x2504, "速度前馈增益",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2505, "速度前馈平滑滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,100,"/"},
{0x2506, "转矩前馈增益",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2507, "转矩前馈滤波时间常数",PARAM_READ_WRITE,PARAM_TYPE_U16,0,100,"/"},
{0x2508, "位置指令一阶滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,6553,"/"},
{0x2509, "位置指令平滑时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      128,   "/"},
{0x2510, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "/"},
{0x2511, "电流指令一阶滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,100,"/"},
{0x2512, "速度反馈一阶滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,20,"/"},
{0x2513, "速度反馈平滑时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      4,     "/"},
{0x2514, "刚性表",          PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      31,    "/"},
{0x2515, "负载惯量比",       PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      120,   "/"},
{0x2516, "位置增益变动比率", PARAM_READ_WRITE, PARAM_TYPE_U16, 10,     500,   "/"},
{0x2517, "速度增益变动比率", PARAM_READ_WRITE, PARAM_TYPE_U16, 10,     500,   "/"},
{0x2518, "增益切换条件",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      255,   "/"},
{0x2519, "增益切换时间",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3000,  "/"},
{0x2520, "增益切换延迟时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3000,  "/"},
{0x2521, "增益切换阈值",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      32767, "/"},
{0x2522, "PDFF控制系数",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2523, "控制环路系数",     PARAM_READ_WRITE, PARAM_TYPE_U16, 10,     100,   "/"},
{0x2524, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      6553,  "/"},
{0x2525, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2526, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2527, "正方向摩擦补偿",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2528, "负方向摩擦补偿",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2529, "摩擦补偿一阶滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,100,"/"},
{0x2530, "第二惯量比",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      120,   "/"},
{0x2531, "性能扩展",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      31,    "/"},

/*********** P6 IO参数 2600 (1个 读写) ***********/
{0x2600, "厂家参数",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      20,    "ms"},

/*********** P7 通信参数 2700~2729 (30个 读写) ***********/
{0x2700, "上位机通信ID",     PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      254,   "/"},
{0x2701, "上位机通讯响应延时",PARAM_READ_WRITE,PARAM_TYPE_U16,1,20,"ms"},
{0x2702, "上位机通信参数5",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2703, "上位机通信参数6",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2704, "上位机通信参数7",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2705, "上位机通信参数8",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2706, "ECAT通信SM数据丢失检测延时",PARAM_READ_WRITE,PARAM_TYPE_U16,0,65535,"/"},
{0x2707, "ECAT通信DC同步模式",PARAM_READ_WRITE,PARAM_TYPE_U16,0,1,"/"},
{0x2708, "ECAT通信参数3",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2709, "ECAT通信参数4",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2710, "ECAT通信参数5",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2711, "ECAT通信参数6",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2712, "ECAT通信参数7",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2713, "ECAT通信参数8",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2714, "CAN通信ID",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      127,   "/"},
{0x2715, "CAN通信波特率",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2716, "CAN通信参数3",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2717, "CAN通信参数4",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2718, "CAN通信参数5",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2719, "CAN通信参数6",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2720, "CAN通信参数7",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2721, "CAN通信参数8",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2722, "Modbus ID",        PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2723, "Modbus 波特率",    PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2724, "Modbus 通信格式",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2725, "Modbus参数4",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2726, "Modbus参数5",      PARAM_READ_WRITE, PARAM_READ_WRITE, 0,      65535, "/"},
{0x2727, "Modbus参数6",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2728, "Modbus参数7",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2729, "Modbus参数8",      PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},

/*********** P8 辅助功能 2800~2830 (31个 读写) ***********/
{0x2800, "软件复位",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2801, "故障复位及编码器复位",PARAM_READ_WRITE,PARAM_TYPE_U16,0,10,"/"},
{0x2802, "点动功能",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      2,     "/"},
{0x2803, "点动速度",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      4000,  "Rpm"},
{0x2804, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2805, "内部使能",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2806, "默认监视项目选择",  PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      63,    "/"},
{0x2807, "软限位开关",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2808, "驱动器过载警告值", PARAM_READ_WRITE, PARAM_TYPE_U16, 20,     100,   "%"},
{0x2809, "电机过载警告值",   PARAM_READ_WRITE, PARAM_TYPE_U16, 20,     100,   "%"},
{0x2810, "点动加减速时间",   PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      30000, "/"},
{0x2811, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 1,      20,    "/"},
{0x2812, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2813, "绝对值使用方式",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      2,     "/"},
{0x2814, "软件过流开关",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2815, "强制抱闸打开",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      256,   "/"},
{0x2816, "高电压抱闸开启工作时间",PARAM_READ_WRITE,PARAM_TYPE_U16,0,3000,"/"},
{0x2817, "厂家参数",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2818, "堵转保护开启电流阈值",PARAM_READ_WRITE,PARAM_TYPE_U16,0,3000,"/"},
{0x2819, "堵转保护检测时间", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "/"},
{0x2820, "堵转保护开关",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x2821, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2822, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2823, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2824, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2825, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2826, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2827, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2828, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2829, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x2830, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},

/*********** P9 振动抑制 2900~2904 (5个 读写) ***********/
{0x2900, "低频抑制震动频率", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "Hz"},
{0x2901, "低频抑制振动增益", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      100,   "/"},
{0x2902, "高频抑制振动频率", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      5000,  "Hz"},
{0x2903, "高频抑制振动宽度", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "/"},
{0x2904, "高频抑制振动深度", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "/"},

/*********** PA 保留参数 3000~3017 (18个 读写) ***********/
{0x3000, "控制模式",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x3001, "位置运动模式",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3,     "/"},
{0x3002, "二级故障停车模式", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x3003, "关使能停车",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x3004, "加速度",           PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      0xFFFFFFFF,"/"},
{0x3005, "减速度",           PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      0xFFFFFFFF,"/"},
{0x3006, "力矩斜坡",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      0xFFFFFFFF,"0.1%/s"},
{0x3007, "力矩模式转速限制", PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      0xFFFFFFFF,"/"},
{0x3008, "位置正极限",       PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000,0x7FFFFFFF,"count"},
{0x3009, "位置负极限",       PARAM_READ_WRITE, PARAM_TYPE_I32, 0x80000,0x7FFFFFFF,"count"},
{0x3010, "位置偏差阈值",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0x80000,0x7FFFFFFF,"count"},
{0x3011, "速度最大值",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      0xFFFFFFFF,"/"},
{0x3012, "力矩最大值",       PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      3000,  "0.001"},
{0x3013, "位置到达精度",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "count"},
{0x3014, "位置到达时间",     PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "ms"},
{0x3015, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3016, "同步通信时间",     PARAM_READ_WRITE, PARAM_TYPE_U16, 4,      65535, "ms"},
{0x3017, "CAN模式反馈帧1返回模式",PARAM_READ_WRITE,PARAM_TYPE_U16,0,2,"/"},

/*********** PB 保留参数 3100 (1个 读写) ***********/
{0x3100, "厂家参数",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},

/*********** PC 保留参数 3200 (1个 读写) ***********/
{0x3200, "厂家参数",         PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},

/*********** PD 全闭环参数 3300~3314 (15个 读写) ***********/
{0x3300, "全闭环功能选择",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x3301, "全闭环控制低通滤波时间",PARAM_READ_WRITE,PARAM_TYPE_U16,1,2048,"/"},
{0x3302, "第二编码器型号",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      16,    "/"},
{0x3303, "第二编码器方向",   PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1,     "/"},
{0x3304, "第二编码器分辨率", PARAM_READ_WRITE, PARAM_TYPE_U32, 0,      0xFFFFFFFF,"/"},
{0x3305, "减速比",           PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      1000,  "/"},
{0x3306, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3307, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3308, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3309, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3310, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "%"},
{0x3311, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "0.1ms"},
{0x3312, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3313, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"},
{0x3314, "保留",             PARAM_READ_WRITE, PARAM_TYPE_U16, 0,      65535, "/"}
};

static int32_t ParamBuffer[PARAM_TOTAL_COUNT];

void JointParam_Init(void) {
    for (uint16_t i = 0; i < PARAM_TOTAL_COUNT; i++) {
        ParamBuffer[i] = ParamTable[i].min;
    }
}

const ParamItemTypeDef* JointParam_FindByAddr(uint16_t addr) {
    for (uint16_t i = 0; i < PARAM_TOTAL_COUNT; i++) {
        if (ParamTable[i].addr == addr) {
            return &ParamTable[i];
        }
    }
    return 0;
}

int32_t JointParam_Read(uint16_t addr) {
    const ParamItemTypeDef* param = JointParam_FindByAddr(addr);
    if (!param) return 0;
    uint16_t idx = param - ParamTable;
    return ParamBuffer[idx];
}

uint8_t JointParam_Write(uint16_t addr, int32_t value) {
    const ParamItemTypeDef* param = JointParam_FindByAddr(addr);
    if (!param) return 0;
    if (param->access == PARAM_READ_ONLY) return 0;
    if (value < param->min || value > param->max) return 0;

    uint16_t idx = param - ParamTable;
    ParamBuffer[idx] = value;
    return 1;
}

uint16_t JointParam_GetCount(void) {
    return PARAM_TOTAL_COUNT;
}

const ParamItemTypeDef* JointParam_GetByIndex(uint16_t index) {
    if (index >= PARAM_TOTAL_COUNT) return 0;
    return &ParamTable[index];
}

void JointParam_LoadDefault(void) {
    for (uint16_t i = 0; i < PARAM_TOTAL_COUNT; i++) {
        ParamBuffer[i] = ParamTable[i].min;
    }
}