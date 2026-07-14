/**
 * @file LoopTiming.h
 * @brief 主循环与上电延时、串口打印节流
 *
 * =============================================================================
 * 【模块职责】
 *   1. setup() 末尾上电稳定等待（默认 1000 ms）
 *   2. loop()  末尾周期延时（默认 2 ms），略降主循环 CPU 占用
 *   3. 串口调试行节流，避免 printDebugInfo 每圈刷屏
 *
 * 【可调宏】
 *   LOOP_SETUP_DELAY_MS   上电等待(ms)；原 setup() delay(1000)
 *   LOOP_PERIOD_DELAY_MS  每圈 loop 末尾延时(ms)；原 loop() delay(2)
 *
 * 【调用链（主程序）】
 *   setup()  → loopTimingDelayAfterSetup()
 *   loop()   → printDebugInfo() 内 loopTimingShouldPrint()
 *           → loopTimingDelayEndOfLoop()
 * =============================================================================
 */
#ifndef LOOP_TIMING_H
#define LOOP_TIMING_H

#include <Arduino.h>
#include "config.h"

/** 上电后等待稳定(ms) */
#define LOOP_SETUP_DELAY_MS  1000
/** 每圈 loop 末尾延时(ms) */
#define LOOP_PERIOD_DELAY_MS 2

/** setup() 末尾调用：上电稳定等待 */
void loopTimingDelayAfterSetup();

/** loop() 末尾调用：略降低主循环频率 */
void loopTimingDelayEndOfLoop();

/** 串口调试行节流；默认间隔 SERIAL_PRINT_MS（config.h） */
bool loopTimingShouldPrint(uint16_t intervalMs = SERIAL_PRINT_MS);

#endif  // LOOP_TIMING_H
