#pragma once
#include "CameraUP.h"
#include "Joints.h"
#include "KineHelper.hpp"
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "RobotException.hpp"
#include "SerialPort.h"

namespace D5R {

struct Points {
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

  D5Robot(const char *serialPort, std::string natorID = "usb:id:7547982319",
          uint8_t topRMDID = 1, uint8_t botRMDID = 2,
          std::string upCameraID = "00-21-49-03-4D-95");
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);
  bool VCJawChange();

  Joints GetCurrentJoint();
  TaskSpace GetCurrentPose();
  // Points FwKine(const Joints j);
  // Joints InvKine(const Points p);

private:
  SerialPort _port;
  bool _isInit;
};
} // namespace D5R
