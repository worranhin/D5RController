#include "D5Robot.h"
#include "ErrorCode.h"
#include "RobotException.hpp"

// #define D5R_EXPORTS
#ifdef D5R_EXPORTS
#define D5R_API __declspec(dllexport)
#else
#define D5R_API __declspec(dllimport)
#endif

#define MAJOR_VERSION 0
#define MINOR_VERSION 2
#define PATCH_VERSION 2

extern "C" {

using namespace D5R;
D5R_API int Test();
D5R_API D5Robot *__stdcall CreateD5RobotInstance(const char *serialPort,
                                                 const char *natorID,
                                                 uint8_t topRMDID,
                                                 uint8_t bottomRMDID);
D5R_API ErrorCode __stdcall DestroyD5RobotInstance(D5Robot *instance);
D5R_API bool __stdcall CallIsInit(D5Robot *instance);
D5R_API ErrorCode __stdcall CallSetZero(D5Robot *instance);
D5R_API ErrorCode __stdcall CallStop(D5Robot *instance);
D5R_API ErrorCode __stdcall CallJointsMoveAbsolute(D5Robot *instance,
                                                   const Joints j);
D5R_API ErrorCode __stdcall CallJointsMoveRelative(D5Robot *instance,
                                                   const Joints j);
D5R_API ErrorCode __stdcall CallGetCurrentJoint(D5Robot *instance, Joints &j);
D5R_API ErrorCode __stdcall D5R_GetLastError();
D5R_API BSTR __stdcall D5R_GetVersion();
}