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
#include "SerialPort.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>
#include <mutex>
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
  PIPARAM _piParam;
  int8_t temperature;
  int16_t power;
  int16_t speed;
  uint16_t encoderValue; // 0~16383

private:
  // const char *_serialPortName;
  SerialPort &_serial;
  uint8_t _id;
  // HANDLE _handle;
  DWORD _bytesRead;
  DWORD _bytesWritten;
  bool _isInit;
  std::mutex _dataMutex;

public:
  // RMDMotor(const char *serialPort, uint8_t id);
  // RMDMotor(HANDLE comHandle, uint8_t id);
  RMDMotor(D5R::SerialPort &serial, uint8_t id);
  ~RMDMotor();
  bool Init();
  bool isInit();
  bool Reconnect();
  bool GetMultiAngle_s(int64_t *angle);
  uint16_t GetSingleAngle_s();
  uint8_t GetHeaderCheckSum(uint8_t *command);
  bool GoAngleAbsolute(int64_t angle);
  bool GoAngleRelative(int64_t angle);
  bool Stop();
  bool SetZero();
  bool GetPI();
  bool WriteAnglePI(const uint8_t *arrPI);
  bool DebugAnglePI(const uint8_t *arrPI);

private:
  uint8_t _checksum(uint8_t nums[], int start, int end);
  bool checkFormat(uint8_t *rxBuffer, uint8_t command, uint8_t dataLen);
};

} // namespace D5R