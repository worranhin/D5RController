#pragma once
#include "CameraUP.h"
#include "KineHelper.hpp"
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

// extern Joints JAWPOINT; // 钳口位置，需要实验确定

class D5Robot {
public:
  NatorMotor natorMotor;
  RMDMotor topRMDMotor;
  RMDMotor botRMDMotor;
  CameraUP upCamera;

  D5Robot(const char *serialPort, std::string natorID = "usb:id:7547982319",
          uint8_t topRMDID = 1, uint8_t botRMDID = 2,
          std::string_view upCameraID = "00-21-49-03-4D-95");
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);
  bool VCJawChange();

  Joints GetCurrentJoint();
  Points GetCurrentPose(); // TODO: implement
  // Points FwKine(const Joints j);
  // Joints InvKine(const Points p);

private:
  SerialPort _port;
  bool _isInit;
};
} // namespace D5R
