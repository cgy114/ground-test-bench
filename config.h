/**
 * @file config.h
 * @brief 台架全局配置（引脚、脉宽、AS5600、输出模式）
 *
 * 【信号总览】
 *   输入：CH3/2/1/4 → D2/D4/D7/D8；AS5600 PWM → D3
 *   输出：旋翼1/2/3 → D5/D6/D10（电调 1000~2000 μs）
 *         偏航 D9 → 电调 1000~2000 μs（与旋翼相同 Servo 脉宽模式）
 *
 * 【RC_PINS 下标】
 *   [0] 油门  CH3 → D2
 *   [1] 偏航  CH4 → D8 → <1500→1000；1500~2000→减500 → D9 电调
 *   [2] 俯仰  CH2 → D4
 *   [3] 滚转  CH1 → D7
 */

#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// AS5600 磁编码器（D3 PWM 测角 + 滤波/预测参数）
// =============================================================================

/** AS5600 PWM 输出接 Arduino 引脚（Uno 上 D3 支持外部中断） */
#define PIN_AS5600_PWM           3

/** |ω| 超过此值(°/s)进入高速预测模式，输出角用外推而非实测 */
#define AS_SPEED_THRESH_HIGH_DEGS  100.0f
/** |ω| 低于此值(°/s)退出高速预测，回到实测/滤波角（与上行构成滞回，避免抖动） */
#define AS_SPEED_THRESH_LOW_DEGS   80.0f
/** 高速预测额外前馈时间(s)；预测角 = θ + ω×(dt + 此值) */
#define AS_TAU_FIXED_S             0.020f
/** 角速度死区(°/s)；|ω| 低于此值强制为 0，抑制静止噪声 */
#define AS_OMEGA_DEADBAND_DEGS     30.0f
/** 角度一阶滤波系数；1.0=完全采用本帧结果，0=完全沿用旧值 */
#define AS_ANGLE_FILTER_ALPHA      1.0f
/** 角速度低通滤波系数；越大越跟本帧差分，越小越平滑 */
#define AS_OMEGA_FILTER_BETA       0.08f

/** 实测角与预测角差超过此值(°) → 本帧丢弃实测、改用预测（angFix=1） */
#define AS_RAW_PRED_MAX_DIFF_DEG   40.0f
/** 实测角相对上一帧 θ1 单帧跳变超过此值(°) → 同上，视为异常帧 */
#define AS_MAX_STEP_PER_FRAME_DEG  50.0f

/** 遗留默认 PWM 周期(μs)；内联版无效帧已不用于硬算角度，仅作参考 */
#define AS_PWM_PERIOD_DEFAULT_US   5000

/** 数据手册 Figure 30：一帧总时钟数 = 128 头 + 4095 数据 + 128 尾 */
#define AS_PWM_FRAME_CLOCKS   4351u
/** 每帧固定高电平头部时钟数（不参与角度编码） */
#define AS_PWM_HEADER_CLOCKS  128u
/** 角度数据区时钟数（0~4095 对应 0°~360°） */
#define AS_PWM_DATA_CLOCKS    4095u

/**
 * PWM 高电平/帧周期（μs）→ 角度（°），含 128 头尾空挡补偿
 * @return 0~360 为有效角； -1 表示 period 无效或 highUs>period
 * 调用方：AS5600Angle.cpp readPwmAngle()
 */
static inline float as5600PwmUsToAngleDeg(unsigned long highUs, unsigned long periodUs) {
  if (periodUs == 0 || highUs > periodUs) {
    return -1.0f;
  }
  float bitTime = (float)periodUs / (float)AS_PWM_FRAME_CLOCKS;
  float dataUs = (float)highUs - (float)AS_PWM_HEADER_CLOCKS * bitTime;
  float dataSpanUs = (float)AS_PWM_DATA_CLOCKS * bitTime;
  if (dataSpanUs <= 0.0f) {
    return -1.0f;
  }
  float angle = 360.0f * dataUs / dataSpanUs;
  if (angle < 0.0f) return 0.0f;
  if (angle > 360.0f) return 360.0f;
  return angle;
}

/**
 * 角度预测总开关（调试混控时可用）
 * 0.0f = 仅用实测角（关闭异常替换与高速/混合预测，angFix 恒为 0）
 * 1.0f = 启用 AS5600Angle::update 完整预测逻辑
 */
#define AS_ANGLE_PREDICT_GAIN  0.0f

// =============================================================================
// 遥控接收机输入（四路 PWM 脉宽，单位 μs）
// =============================================================================

/** 油门 CH3 → Arduino 引脚 */
#define PIN_RC_THROTTLE  2
/** 俯仰 CH2 → Arduino 引脚 */
#define PIN_RC_PITCH     4
/** 滚转 CH1 → Arduino 引脚 */
#define PIN_RC_ROLL      7
/** 偏航 CH4 输入 → Arduino 引脚（电调输出在 PIN_MOTOR_YAW） */
#define PIN_RC_YAW       8

/** 遥控通道数量，与 RC_PINS[] 下标一致 */
#define RC_CHANNEL_COUNT 4
/** pulseIn 等待脉宽超时(μs)；超时返回 0，滤波后沿用上一帧有效值 */
#define RC_TIMEOUT_US    25000
/** 标准舵机/接收机脉宽下限(μs) */
#define RC_MIN_US        1000
/** 标准舵机/接收机脉宽上限(μs) */
#define RC_MAX_US        2000
/** 摇杆中位脉宽(μs) */
#define RC_CENTER_US     1500
/** 上电/丢帧时油门初值(μs)；最低油门，避免误启动 */
#define RC_INIT_THROTTLE_US 1000
/** 上电/丢帧时偏航/俯仰/滚转初值(μs)；中位，使 p/r 初值为 0 */
#define RC_INIT_STICK_US    1500
/** 俯仰/滚转归一化死区(μs)；|脉宽-1500| 小于此值视为 0 杆量 */
#define RC_DEADBAND_US   30
/** 认为有效的最小脉宽(μs)；低于此视为干扰，沿用上一帧 */
#define RC_VALID_MIN_US  900
/** 认为有效的最大脉宽(μs)；高于此视为干扰，沿用上一帧 */
#define RC_VALID_MAX_US  2100
/** 俯仰/滚转归一化下限(μs)；超出则杆量强制为 0（机械限位保护） */
#define RC_STICK_CLAMP_LOW  980
/** 俯仰/滚转归一化上限(μs)；超出则杆量强制为 0 */
#define RC_STICK_CLAMP_HIGH 2020

// =============================================================================
// 电机/偏航输出引脚与混控常量
// =============================================================================

/** 旋翼 1 电调信号引脚（对应混控 outPulseUs[0]） */
#define PIN_MOTOR_ROTOR1 5
/** 旋翼 2 电调信号引脚（对应混控 outPulseUs[1]） */
#define PIN_MOTOR_ROTOR2 6
/** 旋翼 3 电调信号引脚（对应混控 outPulseUs[2]） */
#define PIN_MOTOR_ROTOR3 10
/** 偏航电调信号引脚；Servo 脉宽 1000~2000 μs，与旋翼相同 */
#define PIN_MOTOR_YAW    9

/**
 * 偏航 CH4 脉宽映射偏移(μs)
 * 输入 <1500 → 输出 1000；1500~2000 → 输出 = 输入 - 此值（满杆 1500 μs）
 */
#define YAW_PULSE_OFFSET_US  500

/** 电调最小输出脉宽(μs)；低于此多数电调停转 */
#define ESC_MIN_US 1000
/** 电调最大输出脉宽(μs) */
#define ESC_MAX_US 2000

/** 参与混控的旋翼数量 */
#define ROTOR_COUNT 3
/** 相邻旋翼在机架圆周上的相位间隔(°) */
#define ROTOR_PHASE_SPACING_DEG 120.0f
/** 俯仰/滚转满杆对应的修正脉宽(μs)；p/r = 杆量×此值，杆量 ∈ [-1,1] */
#define ATTITUDE_MIX_US 500.0f

/**
 * 俯仰/滚转混控增益（调试）
 * 1.0f = 正常混控 T_i = t0 + p·sin(θ_i) + r·cos(θ_i)
 * 0.0f = 关闭姿态混控，T_i = t0（仅油门）
 */
#define PITCH_ROLL_MIX_GAIN  1.0f

// =============================================================================
// 调试：串口监测与电调输出模式（详见 README）
// =============================================================================

/**
 * 运行模式
 * 0 = 仅写电调，无串口调试行
 * 1 = 仅串口监测，不写电调（上电安全调试）
 * 2 = 串口 + 电调（务必拆桨或架空）
 */
#define SERIAL_PWM_MONITOR 2
/** 串口调试行最小打印间隔(ms)；避免刷屏占满 USB */
#define SERIAL_PRINT_MS    500

#endif  // CONFIG_H
