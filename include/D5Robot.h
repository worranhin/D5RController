#pragma once
#include "CameraUP.h"
#include "LogUtil.h"
#include "NatorMotor.h"
#include "RMDMotor.h"
#include "RobotException.hpp"
#include "SerialPort.h"

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

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

class D5R_API D5Robot {
public:
  D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
          uint8_t botRMDID, std::string_view upCameraID);
  ~D5Robot();
  bool IsInit();
  bool SetZero();
  bool Stop();
  bool JointsMoveAbsolute(const Joints j);
  bool JointsMoveRelative(const Joints j);
  Points FwKine(const Joints j);
  Joints InvKine(const Points p);

  NatorMotor NatorMotor;
  RMDMotor topRMDMotor;
  RMDMotor botRMDMotor;
  CameraUP upCamera;

private:
  SerialPort _port;
  bool _isInit;
};
} // namespace D5R
