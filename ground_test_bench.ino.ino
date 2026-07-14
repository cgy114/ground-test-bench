/**
 * @file ground_test_bench.ino
 * @brief 复合旋翼台架固件 — PWM 版（AS5600 PWM + 四路遥控 + 三旋翼旋转混控）
 *
 * =============================================================================
 * 【烧录路径】
 *   请打开本文件夹 ground_test_bench.ino\ground_test_bench.ino\ 再上传，
 *   勿与 ground_test_bench\ 旧目录混用（RC_PINS 下标顺序不同）。
 *
 * 【硬件接线速查】
 *   输入：CH3油门→D2  CH4偏航→D8  CH2俯仰→D4  CH1滚转→D7  AS5600→D3
 *   输出：旋翼1/2/3→D5/D6/D10  偏航→D9（Servo 电调 1000~2000 μs，与旋翼相同）
 *
 * 【RC_PINS 与 pulseWidthUs 下标】（本文件定义，全工程必须一致）
 *   [0] 油门  CH3  D2
 *   [1] 偏航  CH4  D8  → 1000~2000 μs → Servo(D9)
 *   [2] 俯仰  CH2  PIN_RC_PITCH     D4  → normalizeStick → ×500 → p
 *   [3] 滚转  CH1  PIN_RC_ROLL      D7  → normalizeStick → ×500 → r
 *
 * 【混控公式】（θ 仅来自 AS5600，不是遥控）
 *   θ1 = as5600Angle.getAngleDeg()（D3 磁编码器）
 *   θ_i = θ1 + i×120°（i=0,1,2）
 *   T_i = t0 + p·sin(θ_i) + r·cos(θ_i)   （单位 μs，再 clamp 到 1000~2000）
 *
 * 【偏航】CH4：<1500→1000 μs；1500~2000→减500输出（1000~1500 μs）→ D9 电调
 *
 * 【模块分工】
 *   config.h       常量
 *   RCInput        遥控读入
 *   MotorControl   电调写出
 *   AS5600Angle    测角 + 预测滤波
 *   LoopTiming     上电/loop 延时、串口打印节流
 *   AS5600.cpp/h   旧类封装（#if 0 禁用，仅参考）
 *
 * 【SERIAL_PWM_MONITOR】见 config.h：0仅电调 1仅串口 2串口+电调
 * =============================================================================
 */

#include "config.h"
#include "RCInput.h"
#include "MotorControl.h"
#include "AS5600Angle.h"
#include "LoopTiming.h"

// ==================== 控制量结构（主程序内部用）====================
struct ControlDemand {
  float t0;  // 油门基准脉宽（μs），= rc.pulseWidthUs[0]
  float p;   // 俯仰修正量（μs），满杆约 ±500
  float r;   // 滚转修正量（μs）
  uint16_t yawPulseUs;  // D9 电调脉宽（μs），与旋翼相同
};

// 全局模块实例
RCInput rcInput;
MotorControl motorControl;
AS5600Angle as5600Angle;

/**
 * 引脚数组顺序 = pulseWidthUs[] 下标顺序
 * 必须与 RCInput.h、config.h 注释一致
 */
const int RC_PINS[RC_CHANNEL_COUNT] = {
  PIN_RC_THROTTLE,  // [0] 油门
  PIN_RC_YAW,       // [1] 偏航输入（电调在 D9）
  PIN_RC_PITCH,     // [2] 俯仰
  PIN_RC_ROLL       // [3] 滚转
};

/**
 * 由机架方位角 θ1 算三桨相位角（度）
 * outDeg[0]=θ1, [1]=θ1+120, [2]=θ1+240
 */
void computeRotorAngles(float theta1Deg, float outDeg[ROTOR_COUNT]) {
  outDeg[0] = normalizeAngle360(theta1Deg);
  outDeg[1] = normalizeAngle360(theta1Deg + ROTOR_PHASE_SPACING_DEG);
  outDeg[2] = normalizeAngle360(theta1Deg + 2.0f * ROTOR_PHASE_SPACING_DEG);
}

/**
 * 三旋翼旋转混控（核心）
 * @param t0  油门基准（μs）
 * @param p,r 俯仰/滚转修正（μs），来自摇杆×ATTITUDE_MIX_US
 * @param rotorAnglesDeg  三桨角度（度），仅 AS5600 决定，遥控不改变 θ
 * @param outPulseUs  三路输出脉宽（未 clamp，由 computeMotorCommand 再限幅）
 */
void mixThreeRotors(float t0, float p, float r,
                    const float rotorAnglesDeg[ROTOR_COUNT],
                    float outPulseUs[ROTOR_COUNT]) {
  for (int i = 0; i < ROTOR_COUNT; i++) {
    float rad = radians(rotorAnglesDeg[i]);
    outPulseUs[i] = t0 + p * PITCH_ROLL_MIX_GAIN * sin(rad)
                        + r * PITCH_ROLL_MIX_GAIN * cos(rad);
  }
}

/**
 * 遥控 → 控制量
 */
ControlDemand readControlDemand(const RCChannels &rc) {
  ControlDemand cmd;
  cmd.t0 = (float)rc.pulseWidthUs[0];

  float pitchStick = rcInput.normalizeStick(rc.pulseWidthUs[2]);
  float rollStick  = rcInput.normalizeStick(rc.pulseWidthUs[3]);
  cmd.p = pitchStick * ATTITUDE_MIX_US;
  cmd.r = rollStick * ATTITUDE_MIX_US;

  cmd.yawPulseUs = MotorControl::ch4PulseUsToYawPulseUs(rc.pulseWidthUs[1]);
  return cmd;
}

/**
 * 控制量 + θ1 → 四路电调脉宽
 */
MotorCommand computeMotorCommand(const ControlDemand &cmd, float theta1Deg) {
  float rotorAngles[ROTOR_COUNT];
  computeRotorAngles(theta1Deg, rotorAngles);

  float rotorPulse[ROTOR_COUNT];
  mixThreeRotors(cmd.t0, cmd.p, cmd.r, rotorAngles, rotorPulse);

  MotorCommand out;
  out.rotor1PulseUs = MotorControl::clampPulseUs((int)rotorPulse[0]);
  out.rotor2PulseUs = MotorControl::clampPulseUs((int)rotorPulse[1]);
  out.rotor3PulseUs = MotorControl::clampPulseUs((int)rotorPulse[2]);
  out.yawPulseUs = cmd.yawPulseUs;
  return out;
}

/**
 * 串口调试行（SERIAL_PWM_MONITOR >= 1 时调用）
 */
void printDebugInfo(const RCChannels &rc, const ControlDemand &cmd,
                    const MotorCommand &motorCmd, float theta1, float omega) {
  if (!loopTimingShouldPrint()) return;

  Serial.print(F("RC T:"));
  Serial.print(rc.pulseWidthUs[0]);
  Serial.print(F(" Y:"));
  Serial.print(rc.pulseWidthUs[1]);
  Serial.print(F(" P:"));
  Serial.print(rc.pulseWidthUs[2]);
  Serial.print(F(" R:"));
  Serial.print(rc.pulseWidthUs[3]);
  Serial.print(F(" | t0:"));
  Serial.print((int)cmd.t0);
  Serial.print(F(" p:"));
  Serial.print((int)cmd.p);
  Serial.print(F(" r:"));
  Serial.print((int)cmd.r);
  Serial.print(F(" yawUs:"));
  Serial.print(cmd.yawPulseUs);
  Serial.print(F(" | R1:"));
  Serial.print(motorCmd.rotor1PulseUs);
  Serial.print(F(" R2:"));
  Serial.print(motorCmd.rotor2PulseUs);
  Serial.print(F(" R3:"));
  Serial.print(motorCmd.rotor3PulseUs);
  Serial.print(F(" YawUs:"));
  Serial.print(motorCmd.yawPulseUs);
  Serial.print(F(" | theta1:"));
  Serial.print(theta1, 1);
  Serial.print(F(" w:"));
  Serial.print(omega, 1);
  Serial.print(F(" angFix:"));
  Serial.print(as5600Angle.lastFrameUsedPredict() ? 1 : 0);
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  rcInput.begin(RC_PINS);
  motorControl.begin();
  as5600Angle.begin();

#if SERIAL_PWM_MONITOR == 0
  Serial.println(F("Mode 0: ESC only"));
#elif SERIAL_PWM_MONITOR == 1
  Serial.println(F("Mode 1: Serial only (no ESC)"));
#else
  Serial.println(F("Mode 2: Serial + ESC (remove props!)"));
#endif
  loopTimingDelayAfterSetup();
}

void loop() {
  rcInput.update();
  const RCChannels &rc = rcInput.get();

  ControlDemand cmd = readControlDemand(rc);

  as5600Angle.update();
  float theta1 = as5600Angle.getAngleDeg();

  MotorCommand motorCmd = computeMotorCommand(cmd, theta1);

#if SERIAL_PWM_MONITOR >= 1
  printDebugInfo(rc, cmd, motorCmd, theta1, as5600Angle.getOmegaDegS());
#endif

#if SERIAL_PWM_MONITOR == 0 || SERIAL_PWM_MONITOR == 2
  motorControl.write(motorCmd);
#endif

  loopTimingDelayEndOfLoop();
}
