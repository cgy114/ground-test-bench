/**
 * @file MotorControl.cpp
 * @brief 四路电调(Servo 脉宽 1000~2000 μs)输出实现
 *
 * =============================================================================
 * 【调用链（主程序）】
 *   setup()  → motorControl.begin()
 *   loop()   → readControlDemand() 内 ch4PulseUsToYawPulseUs()
 *           → computeMotorCommand() 内 clampPulseUs()
 *           → motorControl.write(motorCmd)
 *
 * 【本文件函数】
 *   MotorControl::begin                  四路 Servo attach，偏航先写最低脉宽
 *   MotorControl::clampPulseUs           限幅到 ESC_MIN_US ~ ESC_MAX_US
 *   MotorControl::ch4PulseUsToYawPulseUs CH4 → D9 偏航电调脉宽
 *   MotorControl::write                  一次性写四路电调
 * =============================================================================
 */

#include "MotorControl.h"

/**
 * =============================================================================
 * 【MotorControl::begin】初始化四路电调输出
 *   1. 旋翼 1/2/3 attach 到 D5/D6/D10
 *   2. 偏航 attach 到 D9
 *   3. 偏航先写 ESC_MIN_US（1000），避免上电误启动
 * 调用：setup() → motorControl.begin()
 * =============================================================================
 */
void MotorControl::begin() {
  _servoRotor1.attach(PIN_MOTOR_ROTOR1);  //D5  旋翼 1
  _servoRotor2.attach(PIN_MOTOR_ROTOR2);  //D6  旋翼 2
  _servoRotor3.attach(PIN_MOTOR_ROTOR3);  //D10 旋翼 3
  _servoYaw.attach(PIN_MOTOR_YAW);        //D9  偏航电调

  _servoYaw.writeMicroseconds(ESC_MIN_US);  //上电先最低脉宽，旋翼由首圈 loop write 输出
}

/**
 * =============================================================================
 * 【MotorControl::clampPulseUs】电调脉宽限幅
 *   pulseUs  混控算出的脉宽（可能略超 1000~2000）
 *   返回     钳位到 [ESC_MIN_US, ESC_MAX_US]
 * 调用：computeMotorCommand() 三路旋翼；write() 四路写前再次限幅
 * =============================================================================
 */
uint16_t MotorControl::clampPulseUs(int pulseUs) {
  if (pulseUs < ESC_MIN_US) return ESC_MIN_US;  //低于最低油门
  if (pulseUs > ESC_MAX_US) return ESC_MAX_US;  //高于满油门
  return (uint16_t)pulseUs;
}

/**
 * =============================================================================
 * 【MotorControl::ch4PulseUsToYawPulseUs】偏航 CH4 → D9 电调脉宽
 *   pulseUs  rc.pulseWidthUs[1]，接收机 CH4 原始脉宽
 *
 * 【映射】
 *   < RC_CENTER_US (1500)  → ESC_MIN_US (1000)
 *   1500~2000              → pulseUs - YAW_PULSE_OFFSET_US（500），满杆约 1500 μs
 * 调用：readControlDemand() → cmd.yawPulseUs
 * =============================================================================
 */
uint16_t MotorControl::ch4PulseUsToYawPulseUs(int pulseUs) {
  if (pulseUs < RC_CENTER_US) {
    return ESC_MIN_US;  //偏航杆在中位以下，输出最低
  }
  if (pulseUs > RC_MAX_US) {
    pulseUs = RC_MAX_US;  //钳位到 2000，再减 500
  }
  return (uint16_t)(pulseUs - YAW_PULSE_OFFSET_US);  //1500→1000，2000→1500
}

/**
 * =============================================================================
 * 【MotorControl::write】一次性写入四路电调脉宽
 *   command  MotorCommand：三路旋翼 + 偏航脉宽（μs）
 *   写前对每路 clampPulseUs，再 writeMicroseconds
 * 调用：loop() 末尾；SERIAL_PWM_MONITOR 为 0 或 2 时执行
 * =============================================================================
 */
void MotorControl::write(const MotorCommand &command) {
  _servoRotor1.writeMicroseconds(clampPulseUs(command.rotor1PulseUs));  //D5
  _servoRotor2.writeMicroseconds(clampPulseUs(command.rotor2PulseUs));  //D6
  _servoRotor3.writeMicroseconds(clampPulseUs(command.rotor3PulseUs));  //D10
  _servoYaw.writeMicroseconds(clampPulseUs(command.yawPulseUs));        //D9
}
