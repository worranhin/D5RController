#include "DllApi.h"

D5Robot *CreateD5RobotInstance(const char *serialPort, std::string natorID,
                               uint8_t topRMDID, uint8_t botRMDID) {
  return new D5Robot(serialPort, natorID, topRMDID, botRMDID);
}

void DestroyD5RobotInstance(D5Robot *instance) { delete instance; }

bool CallIsInit(D5Robot *instance) { return instance->IsInit(); }

bool CallSetZero(D5Robot *instance) { return instance->SetZero(); }

bool CallStop(D5Robot *instance) { return instance->Stop(); }

bool CallJointsMoveAbsolute(D5Robot *instance, const Joints j) {
  return instance->JointsMoveAbsolute(j);
}

bool CallJointsMoveRelative(D5Robot *instance, const Joints j) {
  return instance->JointsMoveRelative(j);
}