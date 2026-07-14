/**
 * @file LoopTiming.cpp
 * @brief 主循环与上电延时、串口打印节流（实现）
 *
 * =============================================================================
 * 【调用链（主程序）】
 *   setup()  → loopTimingDelayAfterSetup()
 *   loop()   → printDebugInfo() 内 loopTimingShouldPrint()
 *           → loopTimingDelayEndOfLoop()
 *
 * 【本文件函数】
 *   loopTimingDelayAfterSetup   上电后 delay(LOOP_SETUP_DELAY_MS)
 *   loopTimingDelayEndOfLoop    每圈末尾 delay(LOOP_PERIOD_DELAY_MS)
 *   loopTimingShouldPrint       串口打印节流，间隔默认 SERIAL_PRINT_MS
 * =============================================================================
 */

#include "LoopTiming.h"

/**
 * =============================================================================
 * 【loopTimingDelayAfterSetup】上电稳定等待
 *   延时 LOOP_SETUP_DELAY_MS（默认 1000 ms）
 *   给接收机、传感器、电调初始化留时间，再进入主循环
 * 调用：setup() 末尾，在 rcInput/motorControl/as5600Angle.begin() 之后
 * =============================================================================
 */
void loopTimingDelayAfterSetup() {
  delay(LOOP_SETUP_DELAY_MS);  //阻塞等待，单位 ms
}

/**
 * =============================================================================
 * 【loopTimingDelayEndOfLoop】每圈 loop 末尾周期延时
 *   延时 LOOP_PERIOD_DELAY_MS（默认 2 ms）
 *   略降低主循环频率，减轻 CPU 占用；与 RCInput 四路 pulseIn 阻塞互补
 * 调用：loop() 末尾，motorControl.write() 之后
 * =============================================================================
 */
void loopTimingDelayEndOfLoop() {
  delay(LOOP_PERIOD_DELAY_MS);  //阻塞等待，单位 ms
}

/**
 * =============================================================================
 * 【loopTimingShouldPrint】串口调试行
 *   intervalMs  两次返回 true 的最小间隔(ms)，默认 SERIAL_PRINT_MS
 *   返回        true=本圈允许打印；false=跳过（距上次未满 intervalMs）
 *
 * 【逻辑】
 *   1. 用 static lastPrint 记录上次允许打印的时刻
 *   2. now - lastPrint < intervalMs → false
 *   3. 否则更新 lastPrint 并返回 true
 * 调用：printDebugInfo() 开头，避免串口每圈输出一行
 * =============================================================================
 */
bool loopTimingShouldPrint(uint16_t intervalMs) {
  static unsigned long lastPrint = 0;  //上次允许打印的时刻(ms)，跨圈保持
  unsigned long now = millis();        //当前时刻(ms)

  //距上次打印未满间隔，本圈跳过
  if (now - lastPrint < intervalMs) {
    return false;
  }
  lastPrint = now;  //刷新时刻，本圈允许打印
  return true;
}
