/**
 * @file NatorMotor.h
 * @author drawal (2581478521@qq.com)
 * @brief Nator Motor Class
 * @version 0.2
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "NTControl.h"
#include "RobotException.hpp"
#include <cstdlib>
#include <iostream>
#include <string>
#include <windows.h>

namespace D5R {
struct NTU_Point {
    int x; // 单位: nm
    int y;
    int z;
};

#define NTU_AXIS_X 3 - 1
#define NTU_AXIS_Y 2 - 1
#define NTU_AXIS_Z 1 - 1

class NatorMotor {
  public:
    NatorMotor(std::string id);
    ~NatorMotor();
    void Init();
    void SetZero();
    NTU_Point GetPosition();
    void GoToPoint_A(NTU_Point p);
    void WaitUtilPositioned();
    void GoToPoint_R(NTU_Point p);
    void Stop();

  private:
    NT_INDEX _handle;
    std::string _id;
    unsigned int _status;
};
} // namespace D5R