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
  RMDMotor(const char *serialPort, uint8_t id);
  RMDMotor(HANDLE comHandle, uint8_t id);
  ~RMDMotor();
  bool Init();
  bool isInit();
  bool Reconnect();
  bool GetMultiAngle_s(int64_t *angle);
  uint8_t GetHeaderCheckSum(uint8_t *command);
  bool GoAngleAbsolute(int64_t angle);
  bool GoAngleRelative(int64_t angle);
  bool Stop();
  bool SetZero();
  bool GetPI();
  bool WriteAnglePI(const uint8_t *arrPI);
  bool DebugAnglePI(const uint8_t *arrPI);

  PIPARAM _piParam;

private:
  const char *_serialPort;
  uint8_t _id;
  HANDLE _handle;
  DWORD _bytesRead;
  DWORD _bytesWritten;
  bool _isInit;
};

} // namespace D5R