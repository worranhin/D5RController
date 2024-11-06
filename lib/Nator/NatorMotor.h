/**
 * @file NatorMotor.h
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "SDK/include/NTControl.h"
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
  bool Init();
  bool SetZero();
  bool IsInit();
  bool GetPosition(NTU_Point *p);
  bool GoToPoint_A(NTU_Point p);
  void WaitUtilPositioned();
  bool GoToPoint_R(NTU_Point p);
  bool Stop();

private:
  NT_INDEX _handle;
  std::string _id;
  bool _isInit;
  unsigned int _status;
};
} // namespace D5R