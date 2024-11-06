#include "D5Robot.h"

#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

using namespace D5R;

extern "C" {
D5R_API D5Robot *CreateD5RobotInstance(const char *serialPort,
                                       std::string natorID, uint8_t topRMDID,
                                       uint8_t botRMDID);
D5R_API void DestroyD5RobotInstance(D5Robot *instance);
D5R_API bool CallIsInit(D5Robot *instance);
D5R_API bool CallSetZero(D5Robot *instance);
D5R_API bool CallStop(D5Robot *instance);
D5R_API bool CallJointsMoveAbsolute(D5Robot *instance, const Joints j);
D5R_API bool CallJointsMoveRelative(D5Robot *instance, const Joints j);
}