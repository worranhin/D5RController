#include "D5Robot.h"
#include "ErrorCode.h"

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

using namespace D5R;

extern "C" {
D5R_API ErrorCode CreateD5RobotInstance(D5Robot *instance,
                                        const char *serialPort,
                                        std::string natorID, uint8_t topRMDID,
                                        uint8_t botRMDID);
D5R_API ErrorCode DestroyD5RobotInstance(D5Robot *instance);
D5R_API bool CallIsInit(D5Robot *instance);
D5R_API ErrorCode CallSetZero(D5Robot *instance);
D5R_API ErrorCode CallStop(D5Robot *instance);
D5R_API ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j);
D5R_API ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j);
}