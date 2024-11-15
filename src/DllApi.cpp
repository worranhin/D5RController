#include "DllApi.h"

#define TRY_BLOCK(content)                                                     \
  try {                                                                        \
    content return ErrorCode::OK;                                              \
  } catch (const RobotException &e) {                                          \
    return e.code;                                                             \
  } catch (...) {                                                              \
    return ErrorCode::SystemError;                                             \
  }

ErrorCode CreateD5RobotInstance(D5Robot *instance, const char *serialPort,
                                std::string natorID, uint8_t topRMDID,
                                uint8_t bottomRMDID) {
  TRY_BLOCK(instance =
                new D5Robot(serialPort, natorID, topRMDID, bottomRMDID););
}

ErrorCode DestroyD5RobotInstance(D5Robot *instance) {
  TRY_BLOCK(delete instance;)
}

bool CallIsInit(D5Robot *instance) { return instance->IsInit(); }

ErrorCode CallSetZero(D5Robot *instance) { TRY_BLOCK(instance->SetZero();) }

ErrorCode CallStop(D5Robot *instance) { TRY_BLOCK(instance->Stop();) }

ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j) {
  TRY_BLOCK(instance->JointsMoveAbsolute(j);)
}

ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j) {
  TRY_BLOCK(instance->JointsMoveRelative(j);)
}