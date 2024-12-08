/**
 * @file RMDController.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief RMD motor class
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <Windows.h>
#include <cstdint>
#include <iostream>
#include <thread>

namespace D5R {

struct PIPARAM {
  uint8_t angleKp;
  uint8_t angleKi;
  uint8_t speedKp;
  uint8_t speedKi;
  uint8_t torqueKp;
  uint8_t torqueKi;
};

enum ID_ENTRY {
  ID_01 = (uint8_t)0x01,
  ID_02 = (uint8_t)0x02,
};
class RMDMotor {
public:
  RMDMotor();
  RMDMotor(const char *serialPort, uint8_t id);
  RMDMotor(HANDLE comHandle, uint8_t id);
  ~RMDMotor();
  bool Init();
  bool isInit();
  bool Reconnect();
  bool GetMultiAngle_s(int64_t *angle);
  uint16_t GetSingleAngle_s();
  uint8_t GetHeaderCheckSum(uint8_t *command);
  bool GoAngleAbsolute(int64_t angle);
  bool GoAngleRelative(int64_t angle);
  int16_t OpenLoopControl(int16_t power);
  void SpeedControl(int32_t speed);
  bool Stop();
  bool SetZero();
  bool GetPI();
  bool WriteAnglePI(const uint8_t *arrPI);
  bool DebugAnglePI(const uint8_t *arrPI);

  PIPARAM _piParam;

  HANDLE _handle;
  uint8_t _id;

private:
  const char *_serialPort;
  DWORD _bytesRead;
  DWORD _bytesWritten;
  bool _isInit;
  uint8_t _checksum(uint8_t nums[], int start, int end);
};

} // namespace D5R