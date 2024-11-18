#include "D5Robot.h"

namespace D5R {
D5Robot::D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
                 uint8_t botRMDID)
    : _port(serialPort), natorMotor(natorID),
      topRMDMotor(_port.GetHandle(), topRMDID),
      botRMDMotor(_port.GetHandle(), botRMDID) {
  _isInit = natorMotor.IsInit() && topRMDMotor.isInit() && botRMDMotor.isInit();
  if (!_isInit) {
    throw RobotException(ErrorCode::CreateInstanceError);
  }
}
D5Robot::~D5Robot() {}

bool D5Robot::IsInit() { return _isInit; }

bool D5Robot::SetZero() {
  if (!natorMotor.SetZero()) {
    ERROR_("Failed to set nator motor zero");
    return false;
  }
  if (!topRMDMotor.SetZero()) {
    ERROR_("Failed to set TOP RMD motor zero");
    return false;
  }
  if (!botRMDMotor.SetZero()) {
    ERROR_("Failed to set BOT RMD motor zero");
    return false;
  }
  return true;
}
bool D5Robot::Stop() {
  if (!natorMotor.Stop()) {
    ERROR_("Failed to stop nator motor");
    return false;
  }
  if (!topRMDMotor.Stop()) {
    ERROR_("Failed to stop TOP RMD motor");
    return false;
  }
  if (!botRMDMotor.Stop()) {
    ERROR_("Failed to stop BOT RMD motor");
    return false;
  }
  return true;
}

bool D5Robot::JointsMoveAbsolute(const Joints j) {
  NTU_Point p{j.x, j.y, j.z};
  if (!natorMotor.GoToPoint_A(p)) {
    ERROR_("Failed to move nator motor");
    return false;
  }
  if (!topRMDMotor.GoAngleAbsolute(j.r1)) {
    ERROR_("Failed to move top RMD motor");
    return false;
  }
  if (!botRMDMotor.GoAngleAbsolute(j.r5)) {
    ERROR_("Failed to move bot RMD motor");
    return false;
  }
  return true;
}
bool D5Robot::JointsMoveRelative(const Joints j) {
  NTU_Point p{j.x, j.y, j.z};
  if (!natorMotor.GoToPoint_R(p)) {
    ERROR_("Failed to move nator motor");
    return false;
  }
  if (!topRMDMotor.GoAngleRelative(j.r1)) {
    ERROR_("Failed to move top RMD motor");
    return false;
  }
  if (!botRMDMotor.GoAngleRelative(j.r5)) {
    ERROR_("Failed to move bot RMD motor");
    return false;
  }
  return true;
}

Joints D5Robot::GetCurrentJoint() {
  Joints j;

  j.r1 = topRMDMotor.GetSingleAngle_s();
  j.r5 = botRMDMotor.GetSingleAngle_s();

  NTU_Point np;
  this->natorMotor.GetPosition(&np);
  j.x = np.x;
  j.y = np.y;
  j.z = np.z;

  return j;
}
Pose D5Robot::GetCurrentPose() {
  throw std::logic_error("Not implemented");
  //  return Pose();
}

} // namespace D5R