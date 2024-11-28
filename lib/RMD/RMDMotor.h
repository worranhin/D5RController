/**
 * @file RMDController.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief RMD motor class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "RobotException.hpp"
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
    RMDMotor(HANDLE comHandle, uint8_t id);
    ~RMDMotor();
    int64_t GetMultiAngle_s();
    uint16_t GetSingleAngle_s();
    uint8_t GetHeaderCheckSum(uint8_t *command);
    void GoAngleAbsolute(int64_t angle);
    void GoAngleRelative(int64_t angle);
    int16_t OpenLoopControl(int16_t power);
    void SpeedControl(int32_t speed);
    void Stop();
    void SetZero();
    void GetPI();
    void WriteAnglePI(const uint8_t *arrPI);
    void DebugAnglePI(const uint8_t *arrPI);

    PIPARAM _piParam;

    HANDLE _handle;
    uint8_t _id;

  private:
    DWORD _bytesRead;
    DWORD _bytesWritten;
    uint8_t _checksum(uint8_t nums[], int start, int end);
};

} // namespace D5R