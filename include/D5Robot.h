#pragma once
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "RobotException.hpp"
#include "SerialPort.h"

namespace D5R {

struct Joints {
  int r1;
  int x;
  int y;
  int z;
  int r5;
};

struct Pose {
  double px;
  double py;
  double pz;
  double ry;
  double rz;
};

class D5Robot {
public:
  NatorMotor natorMotor;
  RMDMotor topRMDMotor;
  RMDMotor botRMDMotor;

  D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
          uint8_t botRMDID);
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);
  Joints GetCurrentJoint();
  Pose GetCurrentPose();  // TODO: implement

private:
  SerialPort _port;
  bool _isInit;
};
} // namespace D5R
