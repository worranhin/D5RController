#pragma once
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "SerialPort.h"

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

namespace D5R {

struct Joints {
  int r1;
  int x;
  int y;
  int z;
  int r5;
};

class D5R_API D5Robot {
public:
  D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
          uint8_t botRMDID);
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);

private:
  SerialPort _port;
  NatorMotor _NatorMotor;
  RMDMotor _topRMDMotor;
  RMDMotor _botRMDMotor;
  bool _isInit;
};
} // namespace D5R
