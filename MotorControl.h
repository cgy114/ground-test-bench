/**
 * @file MotorControl.h
 * @brief 四路电机电调输出（旋翼 D5/D6/D10 + 偏航 D9，均为 1000~2000 μs）
 *
 * =============================================================================
 * 【模块职责】
 *   1. 四路 Servo 库输出电调 PWM 脉宽（ESC_MIN_US ~ ESC_MAX_US）
 *   2. clampPulseUs：混控结果限幅到电调可接受范围
 *   3. ch4PulseUsToYawPulseUs：偏航 CH4 杆量 → D9 电调脉宽
 *   4. write：一次性写入旋翼 1/2/3 + 偏航四路输出
 *
 * 【输出引脚】
 *   rotor1PulseUs  D5  PIN_MOTOR_ROTOR1
 *   rotor2PulseUs  D6  PIN_MOTOR_ROTOR2
 *   rotor3PulseUs  D10 PIN_MOTOR_ROTOR3
 *   yawPulseUs     D9  PIN_MOTOR_YAW（与旋翼相同 Servo 脉宽模式）
 *
 * 【偏航映射（ch4PulseUsToYawPulseUs）】
 *   CH4 < 1500              → 1000 μs（最低）
 *   CH4 1500~2000           → 输入减 YAW_PULSE_OFFSET_US（500），满杆约 1500 μs
 *
 * 【调用链（主程序）】
 *   setup()  → motorControl.begin()
 *   loop()   → computeMotorCommand() → motorControl.write(motorCmd)
 *           readControlDemand() 内 ch4PulseUsToYawPulseUs()
 * =============================================================================
 */
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

/** 一帧电机输出：四路电调脉宽（μs） */
struct MotorCommand {
  uint16_t rotor1PulseUs;   // D5  旋翼 1
  uint16_t rotor2PulseUs;   // D6  旋翼 2
  uint16_t rotor3PulseUs;   // D10 旋翼 3
  uint16_t yawPulseUs;      // D9  偏航电调
};

/** 封装四路 Servo 电调，供主 loop 调用 write() 统一输出 */
class MotorControl {
public:
  void begin();
  void write(const MotorCommand &command);
  static uint16_t clampPulseUs(int pulseUs);
  static uint16_t ch4PulseUsToYawPulseUs(int pulseUs);

private:
  Servo _servoRotor1;
  Servo _servoRotor2;
  Servo _servoRotor3;
  Servo _servoYaw;
};

#endif  // MOTORCONTROL_H
