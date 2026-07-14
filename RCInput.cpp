/**
 * @file RCInput.cpp
 * @brief 四通道 pulseIn、滤波、俯仰/滚转杆量归一化（实现）
 *
 * =============================================================================
 * 【调用链（主程序）】
 *   setup()  → rcInput.begin(RC_PINS)
 *   loop()   → rcInput.update() → rcInput.get() → readControlDemand()
 *
 * 【本文件函数】
 *   initPulseUsForChannel  按通道下标返回上电安全初值
 *   RCInput::begin         保存引脚、设 INPUT、填初值
 *   RCInput::filterPulseUs 单通道超时/无效滤波
 *   RCInput::readPulseUs   单路 pulseIn 读脉宽
 *   RCInput::update        每圈读 4 路并写入 _channels
 *   RCInput::normalizeStick 俯仰/滚转脉宽 → [-1, +1]
 * =============================================================================
 */

#include "RCInput.h"

/**
 * =============================================================================
 * 【initPulseUsForChannel】上电安全初值（仅 begin 调用）
 *   [0] 油门              → RC_INIT_THROTTLE_US (1000)，避免误半油门
 *   [1][2][3] 偏航/俯仰/滚转 → RC_INIT_STICK_US    (1500)，p/r 初值为 0
 * =============================================================================
 */
static int initPulseUsForChannel(int channel) {
  if (channel == 0) {
    return RC_INIT_THROTTLE_US;  // 1000
  }
  return RC_INIT_STICK_US;       // 1500
}

/**
 * =============================================================================
 * 【RCInput::begin】初始化四路遥控输入
 *   1. 保存主程序 RC_PINS[] 到 _pins[]
 *   2. 各脚 pinMode INPUT，用于读接收机 PWM
 *   3. _channels 与 _lastValidUs 填入安全初值（第一帧 update 前 get() 也安全）
 * 调用：setup() → rcInput.begin(RC_PINS)
 * =============================================================================
 */
void RCInput::begin(const int pins[RC_CHANNEL_COUNT]) {
  //从 0 到 3，依次初始化四路遥控接收机通道
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    _pins[i] = pins[i];              //把主程序给的引脚号存进 RCInput 内部数组 _pins
    pinMode(_pins[i], INPUT);        //数字输入，读接收机 PWM 高电平脉宽

    //安全初值初始化
    int initUs = initPulseUsForChannel(i);
    _channels.pulseWidthUs[i] = initUs;  //当前帧输出（对外 get()）
    _lastValidUs[i] = initUs;            //超时/无效时回退用的上一帧有效值
  }
}

/**
 * =============================================================================
 * 【RCInput::filterPulseUs】单通道脉宽滤波
 *   channel  0~3，对应油门/偏航/俯仰/滚转
 *   rawUs    readPulseUs 返回值；0=超时，否则为实测脉宽 μs
 *   返回     写入 _channels.pulseWidthUs[channel] 的值
 *
 * 【三分支】
 *   分支1  rawUs==0（超时）           → 返回 _lastValidUs[channel]
 *   分支2  <RC_VALID_MIN 或 >MAX     → 同上，不更新记忆
 *   分支3  有效帧                     → 更新 _lastValidUs，返回 rawUs
 * =============================================================================
 */
int RCInput::filterPulseUs(int channel, int rawUs) {
  //分支1：pulseIn 超时（RC_TIMEOUT_US 内没等到完整高电平，rawUs==0）沿用上一帧有效值；
  if (rawUs == 0) {
    return _lastValidUs[channel];
  }

  //分支2：明显无效脉宽（接线松动、干扰尖峰，<900 或 >2100）
  if (rawUs < RC_VALID_MIN_US || rawUs > RC_VALID_MAX_US) {
    return _lastValidUs[channel];
  }

  //分支3：有效帧，更新记忆并输出实测脉宽
  _lastValidUs[channel] = rawUs;
  return rawUs;
}

/**
 * =============================================================================
 * 【RCInput::readPulseUs】读一路 PWM 高电平宽度
 *   pin    Arduino 数字脚（begin 里已设为 INPUT）
 *   返回   脉宽 μs；超时返回 0（由 filterPulseUs 处理，不在此改成 1500）
 * =============================================================================
 */
int RCInput::readPulseUs(int pin) {
  unsigned long pulse = pulseIn(pin, HIGH, RC_TIMEOUT_US);  //阻塞等待高电平，最长 RC_TIMEOUT_US
  if (pulse == 0) {
    return 0;  //超时
  }
  return (int)pulse;
}

/**
 * =============================================================================
 * 【RCInput::update】每圈 loop 读入四路遥控
 *   顺序对 i=0..3：readPulseUs → filterPulseUs → 写入 _channels
 *   注意：4 路顺序 pulseIn，最坏约 4×RC_TIMEOUT_US；周期延时见 LoopTiming
 * 调用：loop() 每圈一次，之后 rcInput.get() 取 _channels
 * =============================================================================
 */
void RCInput::update() {
  for (int i = 0; i < RC_CHANNEL_COUNT; i++) {
    int raw = readPulseUs(_pins[i]);                      //读第 i 路原始脉宽
    _channels.pulseWidthUs[i] = filterPulseUs(i, raw);  //滤波后写入当前帧输出
  }
}

/**
 * =============================================================================
 * 【RCInput::normalizeStick】俯仰/滚转杆量归一化
 *   pulseWidthUs  通常为 rc.pulseWidthUs[2] 或 [3]
 *   deadband      中位死区，默认 RC_DEADBAND_US（30 μs）
 *   返回          -1.0~+1.0，中位 1500 附近为 0
 *   油门、偏航不在此处理（主程序直接用脉宽或减 500）
 *
 * 【处理步骤】
 *   1. 限位区 pw<980 或 >2020 → 0
 *   2. 钳位到 1000~2000
 *   3. stick = (pw - 1500) / 500
 *   4. |pw-1500| < deadband → 0
 *   5. 限幅到 [-1, +1]
 * =============================================================================
 */
float RCInput::normalizeStick(int pulseWidthUs, float deadband) const {
  int pw = pulseWidthUs;

  //机械限位区异常，直接当中位
  if (pw < RC_STICK_CLAMP_LOW || pw > RC_STICK_CLAMP_HIGH) {
    return 0.0f;
  }
  //钳位到标准舵机范围 1000~2000
  if (pw < RC_MIN_US) {
    pw = RC_MIN_US;
  } else if (pw > RC_MAX_US) {
    pw = RC_MAX_US;
  }

  float stick = (pw - RC_CENTER_US) / 500.0f;  //中位 1500 → 0，满杆约 ±1

  //中位死区：小幅抖动不计入杆量
  if (fabsf(stick) * 500.0f < deadband) {
    return 0.0f;
  }
  //限幅到 [-1, +1]
  if (stick > 1.0f) stick = 1.0f;
  if (stick < -1.0f) stick = -1.0f;
  return stick;
}
