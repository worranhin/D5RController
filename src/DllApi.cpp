#include "DllApi.h"
#include <comdef.h>
#include <comutil.h>

#define TRY_BLOCK(content)              \
    try {                               \
        content return ErrorCode::OK;   \
    } catch (const RobotException &e) { \
        return e.code;                  \
    } catch (...) {                     \
        return ErrorCode::SystemError;  \
    }

ErrorCode CreateD5RobotInstance(D5Robot *&instance, const char *serialPort) {
    try {
        instance = new D5Robot(serialPort);
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::CreateInstanceError;
    }
}

ErrorCode CreateD5RobotInstance2(D5Robot *&instance, const char *serialPort,
                                 const char *natorID, uint8_t topRMDID,
                                 uint8_t bottomRMDID, const char *upCameraID) {
    try {
        instance =
            new D5Robot(serialPort, natorID, topRMDID, bottomRMDID, upCameraID);
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::CreateInstanceError;
    }
}

ErrorCode DestroyD5RobotInstance(D5Robot *instance) {
    // delete instance;
    // return ErrorCode::OK;
    // TRY_BLOCK(delete instance;)
    if (instance == nullptr)
        return ErrorCode::DestroyInstanceError_nullptr;
    try {
        delete instance;
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::CreateInstanceError;
    }
}

ErrorCode InitNator(D5Robot *instance) {
    if (instance == nullptr)
        return ErrorCode::DllCalledWithNullPtr;
    try {
        instance->InitNator();
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::DllError;
    }
}

ErrorCode InitRMD(D5Robot *instance, const char *portName) {
    if (instance == nullptr)
        return ErrorCode::DllCalledWithNullPtr;
    try {
        instance->InitRMD(portName);
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::DllError;
    }
}

ErrorCode InitTopCamera(D5Robot *instance) {
    if (instance == nullptr)
        return ErrorCode::DllCalledWithNullPtr;
    try {
        instance->InitTopCamera();
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::DllError;
    }
}

ErrorCode InitBotCamera(D5Robot *instance) {
    if (instance == nullptr)
        return ErrorCode::DllCalledWithNullPtr;
    try {
        instance->InitBotCamera();
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    } catch (...) {
        return ErrorCode::DllError;
    }
}

ErrorCode CallSetZero(D5Robot *instance) { TRY_BLOCK(instance->SetZero();) }

ErrorCode CallStop(D5Robot *instance) { TRY_BLOCK(instance->Stop();) }

ErrorCode CallJointsMoveAbsolute(D5Robot *instance, const Joints j) {
    TRY_BLOCK(instance->JointsMoveAbsolute(j);)
}

ErrorCode CallJointsMoveRelative(D5Robot *instance, const Joints j) {
    TRY_BLOCK(instance->JointsMoveRelative(j);)
}

ErrorCode CallTaskMoveAbsolute(D5Robot *instance, const TaskSpace ts) {
    try {
        instance->TaskMoveAbsolute(ts);
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    }
}

ErrorCode CallTaskMoveRelative(D5Robot *instance, const TaskSpace ts) {
    try {
        instance->TaskMoveRelative(ts);
        return ErrorCode::OK;
    } catch (const RobotException &e) {
        return e.code;
    }
}

ErrorCode CallGetTopCameraImg(D5Robot *instance) {
    return ErrorCode::UnimplementError;
    // try {
    //     cv::OutputArray output();
    //     //instance->topCamera->Read();
    //     return ErrorCode::OK;
    // } catch (const RobotException &e) {
    //     return e.code;
    // }
}

BSTR D5R_GetVersion() {
    static std::string version = std::to_string(MAJOR_VERSION) + "." +
                                 std::to_string(MINOR_VERSION) + "." +
                                 std::to_string(PATCH_VERSION);

    return _com_util::ConvertStringToBSTR(version.c_str());
}