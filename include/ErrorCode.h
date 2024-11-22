#pragma once

namespace D5R {

enum ErrorCode {
    OK = 0,
    SystemError = 100,
    CreateInstanceError = 101,
    DestroyInstanceError_nullptr,
    SerialError = 200,
    SerialInitError = 201,
    SerialCloseError = 202,
    SerialSendError = 203,
    SerialReceiveError,
    SerialReceiveError_LessThanExpected,
    SerialClearBufferError,
    NatorError = 300,
    NatorInitError = 301,
    NatorMoveError,
    RMDError = 400,
    RMDInitError = 401,
    RMDGetPIError = 402,
    RMDFormatError,
    RMDChecksumError,
    RMDMoveError,
    D5RError = 500,
    D5RMoveError,
    D5RCameraNotInitialized,
    CameraError = 600,
    CameraInitError,
    CameraReadError
};
}