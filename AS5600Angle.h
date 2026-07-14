/**
 * @file AS5600Angle.h
 * @brief AS5600 PWM 测角 + 角速度估计 + 预测滤波（当前固件使用）
 *
 * =============================================================================
 * 【模块职责】
 *   1. D3 双边沿中断采样 AS5600 PWM 高电平时间与帧周期
 *   2. config.h as5600PwmUsToAngleDeg() 换算角度（含 128 头尾空挡补偿）
 *   3. 角速度 ω 估计 + 低通滤波
 *   4. 异常帧剔除、高速/混合预测（受 AS_ANGLE_PREDICT_GAIN 控制）
 *
 * 【对外接口】
 *   begin()                 配置 D3、注册中断、读初始角
 *   update()                每圈读 PWM → 滤波/预测 → 更新 θ1 与 ω
 *   getAngleDeg()           当前方位角 θ1（°），供混控 computeRotorAngles
 *   getOmegaDegS()          角速度 ω（°/s），串口调试
 *   lastFrameUsedPredict()  本帧是否用预测角替代实测（串口 angFix）
 *
 * 【预测开关】config.h AS_ANGLE_PREDICT_GAIN
 *   0.0f = 仅用实测角，angFix 恒为 0
 *   1.0f = 完整预测逻辑（异常帧替换 + 高速外推 + 混合模式）
 *
 * 【调用链（主程序）】
 *   setup()  → as5600Angle.begin()
 *   loop()   → as5600Angle.update()
 *           → theta1 = as5600Angle.getAngleDeg()
 *           → computeMotorCommand(cmd, theta1)
 * =============================================================================
 */
#ifndef AS5600_ANGLE_H
#define AS5600_ANGLE_H

#include <Arduino.h>
#include "config.h"

/** 角度归一化到 [0, 360)；混控 computeRotorAngles 亦使用 */
float normalizeAngle360(float angleDeg);

class AS5600Angle {
public:
  void begin();
  void update();

  float getAngleDeg() const { return _angleDeg; }
  float getOmegaDegS() const { return _omegaDegS; }
  bool lastFrameUsedPredict() const { return _lastFrameUsedPredict; }

private:
  static void pwmIsr();
  void onPwmEdge();
  float readPwmAngle();
  float predictAngleFromState(float dt) const;
  static float shortestPathDeltaDeg(float fromDeg, float toDeg);

  volatile unsigned long _pwmStartTime;   //双边沿采样：边沿时刻
  volatile unsigned long _pwmHighTime;    //本帧高电平宽度(μs)
  volatile unsigned long _pwmPeriodUs;    //本帧周期(μs)
  volatile bool _pwmUpdated;              //中断已采到新帧

  float _angleDeg;              //滤波后方位角 θ1（°）
  float _omegaDegS;             //角速度 ω（°/s）
  unsigned long _lastMicros;    //上一帧 update 时刻(μs)
  bool _lastFrameUsedPredict;   //本帧 angFix 标志

  static AS5600Angle *_instance;  //中断回调用单例指针
};

#endif  // AS5600_ANGLE_H
