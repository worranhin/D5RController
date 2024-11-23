#pragma once

#include "ErrorCode.h"
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
      std::string whatInfo; // 注意这里的字符串不能以 e/E 开头，否则会乱码，原因不明
      if (!this->msg.empty()) {
          whatInfo = (this->msg + " Errorcode: " + std::to_string(this->code));
      } else {
          whatInfo = ("The Error code: " + std::to_string(this->code)); // 注意这里的字符串不能以 e/E 开头，否则会乱码，原因不明
      }

      // *_whatInfo = whatInfo.c_str();

      return whatInfo.c_str();

      // std::string str;
      // switch (code) {
      // case ErrorCode::OK:
      //     return "OK";
      // case ErrorCode::SystemError:
      //     return "System Error";
      // case ErrorCode::CreateInstanceError:
      //     return "Create Instance Error";
      // case ErrorCode::SerialError:
      //     return "Serial Error";
      // case ErrorCode::SerialInitError:
      //     return "Serial Init Error";
      // case ErrorCode::NatorError:
      //     return "Nator Error";
      // case ErrorCode::RMDError:
      //     return "RMD Error";
      // case ErrorCode::CameraError:
      //     return "Camera Error";
      // default:
      //     std::string s = "Other Error: " + std::to_string(code);
      //     return s.c_str();
      // }
      // return str.c_str();
  }
};

} // namespace D5R