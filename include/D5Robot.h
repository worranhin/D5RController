#pragma once
#include "CameraUP.h"
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "RobotException.hpp"
#include "SerialPort.h"

namespace D5R {

struct Joints {
  int r1;
  int p2;
  int p3;
  int p4;
  int r5;
};
struct Points {
  double px;
  double py;
  double pz;
  double ry;
  double rz;
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
  CameraUP upCamera;

  D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
          uint8_t botRMDID, std::string_view upCameraID);
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);

  Joints GetCurrentJoint();
  Pose GetCurrentPose();  // TODO: implement
  Points FwKine(const Joints j);
  Joints InvKine(const Points p);
  
private:
  SerialPort _port;
  bool _isInit;
};
} // namespace D5R
