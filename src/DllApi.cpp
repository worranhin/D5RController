#include "DllApi.h"

ErrorCode CreateD5RobotInstance(D5Robot *instance, const char *serialPort,
                                std::string natorID, uint8_t topRMDID,
                                uint8_t botRMDID) {

  try {
    instance = new D5Robot(serialPort, natorID, topRMDID, botRMDID);
    return ErrorCode::OK;
  } catch (ErrorCode &e) {
    return e;
  } catch (...) {
    return ErrorCode::CreateInstanceError;
  }
}

ErrorCode DestroyD5RobotInstance(D5Robot *instance) {
  try {
    delete instance;
    return ErrorCode::OK;
  } catch (const std::exception &e) {
    // std::cerr << e.what() << '\n';
    return ErrorCode::SystemError;
  }
}

bool CallIsInit(D5Robot *instance) { return instance->IsInit(); }

ErrorCode CallSetZero(D5Robot *instance) {
  instance->SetZero();
  return ErrorCode::OK;
}

ErrorCode CallStop(D5Robot *instance) {
  instance->Stop();
  return ErrorCode::OK;
}

ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j) {
  instance->JointsMoveAbsolute(j);
  return ErrorCode::OK;
}

ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j) {
  instance->JointsMoveRelative(j);
  return ErrorCode::OK;
}