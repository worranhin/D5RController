#pragma once

namespace D5R {

enum ErrorCode {
  OK = 0,
  SystemError = 100,
  CreateInstanceError = 101,
  SerialError = 200,
  SerialInitError = 201,
  SerialCloseError = 202,
  NatorError = 300,
  RMDError = 400
};

}