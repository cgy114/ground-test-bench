/**
 * @file AS5600Angle.cpp
 * @brief AS5600 PWM 测角 + 预测滤波（实现）
 *  方位角测量模块
 * =============================================================================
 * 【调用链（主程序）】
 *   setup()  → as5600Angle.begin()
 *   loop()   → as5600Angle.update() → getAngleDeg() / getOmegaDegS()
 *
 * 【本文件函数】
 *   normalizeAngle360           角度归一化到 [0, 360)
 *   shortestPathDeltaDeg        两角最短路径差值（±180° 内）
 *   pwmIsr / onPwmEdge          D3 双边沿中断采样 PWM
 *   begin                       初始化引脚与中断
 *   readPwmAngle                读中断缓存并换算角度
 *   predictAngleFromState       θ + ω×(dt+τ) 外推
 *   update                      滤波/预测主流程
 * =============================================================================
 */

#include "AS5600Angle.h"
#include <math.h>

AS5600Angle *AS5600Angle::_instance = nullptr;

/**
 * =============================================================================
 * 【normalizeAngle360】角度归一化到 [0, 360)
 *   混控 computeRotorAngles 与预测角计算均使用
 * =============================================================================
 */
float normalizeAngle360(float angleDeg) {
  angleDeg = fmod(angleDeg, 360.0f);
  if (angleDeg < 0) angleDeg += 360.0f;
  return angleDeg;
}

/**
 * =============================================================================
 * 【shortestPathDeltaDeg】两角最短路径差值
 *   返回 toDeg - fromDeg，结果在 (-180, 180] 内
 *   用于角速度差分、实测与预测角比较，避免 359°→1° 误判为大跳变
 * =============================================================================
 */
float AS5600Angle::shortestPathDeltaDeg(float fromDeg, float toDeg) {
  float d = toDeg - fromDeg;
  if (d > 180.0f) d -= 360.0f;
  else if (d < -180.0f) d += 360.0f;
  return d;
}

/**
 * =============================================================================
 * 【pwmIsr】D3 外部中断入口（CHANGE 双边沿）
 *   通过 _instance 转发到 onPwmEdge()
 * =============================================================================
 */
void AS5600Angle::pwmIsr() {
  if (_instance != nullptr) {
    _instance->onPwmEdge();
  }
}

/**
 * =============================================================================
 * 【onPwmEdge】双边沿采样 PWM 高电平与周期
 *   上升沿：记录边沿时刻，用两次上升沿间隔作为 period
 *   下降沿：计算 highTime = 下降沿 - 最近一次上升沿，置 _pwmUpdated
 * =============================================================================
 */
void AS5600Angle::onPwmEdge() {
  unsigned long now = micros();
  if (digitalRead(PIN_AS5600_PWM) == HIGH) {
    //上升沿：更新周期（若已有上一上升沿）
    if (_pwmStartTime != 0) {
      _pwmPeriodUs = now - _pwmStartTime;
    }
    _pwmStartTime = now;
  } else {
    //下降沿：计算高电平宽度，标记新帧可用
    if (_pwmStartTime != 0) {
      _pwmHighTime = now - _pwmStartTime;
      _pwmUpdated = true;
    }
  }
}

/**
 * =============================================================================
 * 【AS5600Angle::begin】初始化测角模块
 *   1. 清零 PWM 采样与角度状态
 *   2. D3 pinMode INPUT，attachInterrupt CHANGE
 *   3. 读一次初始角，记录 _lastMicros
 * 调用：setup() → as5600Angle.begin()
 * =============================================================================
 */
void AS5600Angle::begin() {
  _pwmStartTime = 0;
  _pwmHighTime = 0;
  _pwmPeriodUs = 0;
  _pwmUpdated = false;
  _angleDeg = 0.0f;
  _omegaDegS = 0.0f;
  _lastMicros = 0;
  _lastFrameUsedPredict = false;

  pinMode(PIN_AS5600_PWM, INPUT);  //D3 AS5600 PWM 输入
  _instance = this;                //供 pwmIsr 回调
  attachInterrupt(digitalPinToInterrupt(PIN_AS5600_PWM), pwmIsr, CHANGE);

  _angleDeg = readPwmAngle();      //首帧角度（可能沿用 0 或中断已采到的值）
  _lastMicros = micros();
}

/**
 * =============================================================================
 * 【readPwmAngle】从 volatile 缓存读角并换算
 *   无新帧(_pwmUpdated==false) → 沿用 _angleDeg
 *   有新帧 → noInterrupts 拷贝 ht/period，as5600PwmUsToAngleDeg 换算
 *   无效帧(angle<0) → 沿用 _angleDeg
 * =============================================================================
 */
float AS5600Angle::readPwmAngle() {
  if (!_pwmUpdated) {
    return _angleDeg;  //本圈尚无新 PWM 帧
  }

  unsigned long ht;
  unsigned long period;
  noInterrupts();           //原子拷贝，避免中断中途改写
  _pwmUpdated = false;
  ht = _pwmHighTime;
  period = _pwmPeriodUs;
  interrupts();

  float angle = as5600PwmUsToAngleDeg(ht, period);  //config.h，含 128 头尾补偿
  if (angle < 0.0f) {
    return _angleDeg;  //period 无效等，沿用上一角度
  }
  return angle;
}

/**
 * =============================================================================
 * 【predictAngleFromState】由当前 θ、ω 外推预测角
 *   公式：θ + ω × (dt + AS_TAU_FIXED_S)，再归一化到 [0, 360)
 * =============================================================================
 */
float AS5600Angle::predictAngleFromState(float dt) const {
  float p = _angleDeg + _omegaDegS * (dt + AS_TAU_FIXED_S);
  return normalizeAngle360(p);
}

/**
 * =============================================================================
 * 【AS5600Angle::update】每圈 loop 测角与滤波/预测主流程
 *
 * 【步骤】
 *   1. 算 dt；异常 dt 则跳过本帧
 *   2. readPwmAngle → rawAngle
 *   3. 异常帧判定：与预测角或上一帧差过大 → 用预测角（angFix=1）
 *   4. 差分估 ω，死区 + 低通滤波
 *   5. 按 AS_ANGLE_PREDICT_GAIN 与 |ω| 选 finalAngle（实测/预测/混合）
 *   6. 角度一阶滤波写入 _angleDeg
 * 调用：loop() 每圈一次，之后 getAngleDeg() 供混控
 * =============================================================================
 */
void AS5600Angle::update() {
  unsigned long now = micros();
  float dt = (now - _lastMicros) / 1e6f;
  //dt 异常（首帧、长时间卡顿）则只刷新时刻，不更新角度
  if (dt <= 0.0f || dt > 0.1f) {
    _lastMicros = now;
    return;
  }

  float rawAngle = readPwmAngle();
  rawAngle = normalizeAngle360(rawAngle);

  float predAngle = predictAngleFromState(dt);
  float diffPred = fabsf(shortestPathDeltaDeg(predAngle, rawAngle));
  float diffLast = fabsf(shortestPathDeltaDeg(_angleDeg, rawAngle));

  //异常帧剔除：实测与预测/上一帧差过大时改用预测角
  _lastFrameUsedPredict = false;
  float angleForUpdate = rawAngle;
  if (AS_ANGLE_PREDICT_GAIN > 0.0f
      && (diffPred > AS_RAW_PRED_MAX_DIFF_DEG || diffLast > AS_MAX_STEP_PER_FRAME_DEG)) {
    angleForUpdate = predAngle;
    _lastFrameUsedPredict = true;  //串口 angFix:1
  }

  //角速度：最短路径差分 / dt，死区抑制静止噪声
  float deltaAngle = shortestPathDeltaDeg(_angleDeg, angleForUpdate);
  float rawOmega = deltaAngle / dt;
  if (fabs(rawOmega) < AS_OMEGA_DEADBAND_DEGS) rawOmega = 0.0f;
  _omegaDegS = AS_OMEGA_FILTER_BETA * rawOmega
             + (1.0f - AS_OMEGA_FILTER_BETA) * _omegaDegS;

  //finalAngle：预测关→纯实测；高速→外推；低速→angleForUpdate；中间→混合
  float finalAngle;
  if (AS_ANGLE_PREDICT_GAIN <= 0.0f) {
    finalAngle = rawAngle;
  } else if (fabs(_omegaDegS) > AS_SPEED_THRESH_HIGH_DEGS) {
    float predict = _angleDeg + _omegaDegS * (dt + AS_TAU_FIXED_S);
    predict = fmod(predict, 360.0f);
    if (predict < 0) predict += 360.0f;
    finalAngle = predict;
  } else if (fabs(_omegaDegS) < AS_SPEED_THRESH_LOW_DEGS) {
    finalAngle = angleForUpdate;
  } else {
    //滞回区间内：实测与预测线性混合
    float t = (fabs(_omegaDegS) - AS_SPEED_THRESH_LOW_DEGS)
            / (AS_SPEED_THRESH_HIGH_DEGS - AS_SPEED_THRESH_LOW_DEGS);
    float pred = predictAngleFromState(dt);
    finalAngle = (1.0f - t) * angleForUpdate + t * pred;
    finalAngle = fmod(finalAngle, 360.0f);
    if (finalAngle < 0) finalAngle += 360.0f;
  }

  //角度一阶滤波（AS_ANGLE_FILTER_ALPHA=1 时等价于直接采用 finalAngle）
  _angleDeg = AS_ANGLE_FILTER_ALPHA * finalAngle
            + (1.0f - AS_ANGLE_FILTER_ALPHA) * _angleDeg;
  _angleDeg = fmod(_angleDeg, 360.0f);
  if (_angleDeg < 0) _angleDeg += 360.0f;

  _lastMicros = now;
}
