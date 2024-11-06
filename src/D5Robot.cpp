#include "D5Robot.h"

namespace D5R {
D5Robot::D5Robot(const char *serialPort, std::string natorID, uint8_t topRMDID,
                 uint8_t botRMDID)
    : _port(serialPort), 
      _NatorMotor(natorID),
      _topRMDMotor(_port.GetHandle(), topRMDID),
      _botRMDMotor(_port.GetHandle(), botRMDID) {
  _isInit =
      _NatorMotor.IsInit() && _topRMDMotor.isInit() && _botRMDMotor.isInit();
      
}
D5Robot::~D5Robot() {}

bool D5Robot::IsInit() { return _isInit; }

bool D5Robot::SetZero() {
  if (!_NatorMotor.SetZero()) {
    ERROR_("Failed to set nator motor zero");
    return false;
  }
  if (!_topRMDMotor.SetZero()) {
    ERROR_("Failed to set TOP RMD motor zero");
    return false;
  }
  if (!_botRMDMotor.SetZero()) {
    ERROR_("Failed to set BOT RMD motor zero");
    return false;
  }
  return true;
}
bool D5Robot::Stop() {
  if (!_NatorMotor.Stop()) {
    ERROR_("Failed to stop nator motor");
    return false;
  }
  if (!_topRMDMotor.Stop()) {
    ERROR_("Failed to stop TOP RMD motor");
    return false;
  }
  if (!_botRMDMotor.Stop()) {
    ERROR_("Failed to stop BOT RMD motor");
    return false;
  }
  return true;
}

bool D5Robot::JointsMoveAbsolute(const Joints j) {
  NTU_Point p{j.x, j.y, j.z};
  if (!_NatorMotor.GoToPoint_A(p)) {
    ERROR_("Failed to move nator motor");
    return false;
  }
  if (!_topRMDMotor.GoAngleAbsolute(j.r1)) {
    ERROR_("Failed to move top RMD motor");
    return false;
  }
  if (!_botRMDMotor.GoAngleAbsolute(j.r5)) {
    ERROR_("Failed to move bot RMD motor");
    return false;
  }
  return true;
}
bool D5Robot::JointsMoveRelative(const Joints j) {
  NTU_Point p{j.x, j.y, j.z};
  if (!_NatorMotor.GoToPoint_R(p)) {
    ERROR_("Failed to move nator motor");
    return false;
  }
  if (!_topRMDMotor.GoAngleRelative(j.r1)) {
    ERROR_("Failed to move top RMD motor");
    return false;
  }
  if (!_botRMDMotor.GoAngleRelative(j.r5)) {
    ERROR_("Failed to move bot RMD motor");
    return false;
  }
  return true;
}

} // namespace D5R