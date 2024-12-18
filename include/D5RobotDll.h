#include "D5Robot.h"
#include "ErrorCode.h"
#include "RobotException.hpp"

// #define D5R_EXPORTS
#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

extern "C" {

using namespace D5R;
D5R_API int Test();
D5R_API D5Robot* CreateD5RobotInstance(const char *serialPort,
                                const char *natorID, uint8_t topRMDID,
                                uint8_t bottomRMDID);
D5R_API ErrorCode DestroyD5RobotInstance(D5Robot *instance);
D5R_API bool CallIsInit(D5Robot *instance);
D5R_API ErrorCode CallSetZero(D5Robot *instance);
D5R_API ErrorCode CallStop(D5Robot *instance);
D5R_API ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j);
D5R_API ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j);
D5R_API ErrorCode D5R_GetLastError();
D5R_API BSTR D5R_GetVersion();

}