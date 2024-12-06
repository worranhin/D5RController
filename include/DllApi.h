#include "D5Robot.h"
#include "RobotException.hpp"

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

using namespace D5R;

extern "C" {
D5R_API ErrorCode CreateD5RobotInstance(D5Robot *&instance,
                                        const char *serialPort);
D5R_API ErrorCode CreateD5RobotInstance2(D5Robot *&instance,
                                         const char *serialPort,
                                         const char *natorID, uint8_t topRMDID,
                                         uint8_t bottomRMDID,
                                         const char *upCameraID);
D5R_API ErrorCode DestroyD5RobotInstance(D5Robot *instance);
D5R_API ErrorCode CallSetZero(D5Robot *instance);
D5R_API ErrorCode CallStop(D5Robot *instance);
D5R_API ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j);
D5R_API ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j);
D5R_API ErrorCode CallTaskMoveAbsolute(D5Robot *instance, const TaskSpace ts);
D5R_API ErrorCode CallTaskMoveRelative(D5Robot *instance, const TaskSpace ts);
}