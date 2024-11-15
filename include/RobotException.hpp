#pragma once

#include "ErrorCode.h"
#include <exception>
#include <string>

namespace D5R {

class RobotException : public std::exception {
public:
  ErrorCode code;
  RobotException() noexcept {};
  RobotException(ErrorCode code) noexcept { this->code = code; }
  RobotException(const RobotException &other) noexcept : code(other.code) {}
  RobotException &operator=(const RobotException &other) noexcept {
    this->code = other.code;
    return *this;
  }

  const char *what() const noexcept {
    switch (code) {
    case ErrorCode::OK:
      return "OK";
    case ErrorCode::SystemError:
      return "System Error";
    case ErrorCode::CreateInstanceError:
      return "Create Instance Error";
    case ErrorCode::SerialError:
      return "Serial Error";
    case ErrorCode::SerialInitError:
      return "Serial Init Error";
    case ErrorCode::NatorError:
      return "Nator Error";
    case ErrorCode::RMDError:
      return "RMD Error";
    default:
      std::string s = "Unknown Error: " + std::to_string(code);
      return s.c_str();
    }
  }
};

} // namespace D5R