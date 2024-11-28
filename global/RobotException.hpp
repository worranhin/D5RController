#pragma once

#include "ErrorCode.h"
#include "LogUtil.h"
#include <exception>
#include <string>

namespace D5R {

class RobotException : public std::exception {
  public:
    ErrorCode code;
    std::string msg;
    RobotException() noexcept {};
    RobotException(ErrorCode code) noexcept { this->code = code; }
    RobotException(ErrorCode code, std::string msg) noexcept {
        this->code = code;
        this->msg = msg;
    }
    RobotException(const RobotException &other) noexcept : code(other.code) {}
    RobotException &operator=(const RobotException &other) noexcept {
        this->code = other.code;
        return *this;
    }

    const char *what() const noexcept {
        std::string whatInfo;
        if (!this->msg.empty()) {
            whatInfo = (this->msg + " Errorcode: " + std::to_string(this->code));
        } else {
            whatInfo = ("The Error code: " + std::to_string(this->code)); // 注意这里的字符串不能以 e/E 开头，否则可能会乱码，原因不明
        }

        return whatInfo.c_str();
    }
};

} // namespace D5R