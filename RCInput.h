/**
 * @file RCInput.h
 * @brief 接收机四通道 PWM 输入模块（头文件）
 *
 * =============================================================================
 * 【模块职责】
 *   1. 用 pulseIn() 读取 4 路遥控脉宽（典型 1000~2000 μs，50Hz）
 *   2. 对超时、杂波做滤波（沿用上一帧数值）
 *   3. 提供 normalizeStick()，把俯仰/滚转脉宽变成 -1~+1 杆量
 *
 * 【pulseWidthUs[] 遥控接收机每一路 PWM 信号的高电平脉宽，与主程序 RC_PINS 顺序一致】
 *   [0] 油门  接收机 CH3 → Arduino D2  → 主程序直接作 t0，不经 normalizeStick
 *   [1] 偏航  CH4 → D8  → <1500→1000；1500~2000→减500 → D9 电调
 *   [2] 俯仰  接收机 CH2 → Arduino D4  → normalizeStick → p
 *   [3] 滚转  接收机 CH1 → Arduino D7  → normalizeStick → r
 *
 * 【滤波策略摘要】（实现见 RCInput.cpp）
 *   超时 rawUs==0     → 返回 _lastValidUs[channel]（上一帧有效值）
 *   无效 <900 或 >2100 → 同上，不更新
 *   上电初值           → [0]=1000，[1][2][3]=1500（见 config.h RC_INIT_*）
 * =============================================================================
 */
#ifndef RCINPUT_H
#define RCINPUT_H

#include <Arduino.h>
#include "config.h"

/** 一帧遥控数据：4 路脉宽，单位 μs */
struct RCChannels {
  int pulseWidthUs[RC_CHANNEL_COUNT];
};

class RCInput {
public:
  /**
   * 初始化：保存引脚号、设为 INPUT、设置初值与 _lastValidUs
   * @param pins 长度 RC_CHANNEL_COUNT，顺序同主程序 RC_PINS[]
   */
  void begin(const int pins[RC_CHANNEL_COUNT]);

  /**
   * 顺序读取 4 路 pulseIn（每路最多阻塞 RC_TIMEOUT_US），滤波后写入 _channels
   * 主循环每圈调用一次
   */
  void update();

  /** 只读引用，获取最新滤波后的脉宽 */
  const RCChannels& get() const { return _channels; }

  /**
   * 俯仰/滚转杆量归一化
   * @param pulseWidthUs  通常为 rc.pulseWidthUs[2] 或 [3]
   * @param deadband      中位死区，默认 RC_DEADBAND_US（30 μs）
   * @return -1.0~+1.0，中位 1500 附近为 0
   */
  float normalizeStick(int pulseWidthUs, float deadband = RC_DEADBAND_US) const;

private:
  int _pins[RC_CHANNEL_COUNT];         // 各通道对应的 Arduino 引脚号
  RCChannels _channels;                // 当前帧输出给主程序的脉宽
  int _lastValidUs[RC_CHANNEL_COUNT];  // 每通道“上一帧有效脉宽”，超时/无效时回退用它

  int readPulseUs(int pin);                  // 单路 pulseIn，超时返回 0
  int filterPulseUs(int channel, int rawUs); // 超时/无效/有效 三分支
};

#endif  // RCINPUT_H
