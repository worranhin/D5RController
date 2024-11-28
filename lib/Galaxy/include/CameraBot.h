/**
 * @file CameraBot.h
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "GalaxyCamera.h"

namespace D5R {

class CameraBot : public GxCamera {

  public:
    CameraBot(std::string id);
    ~CameraBot();

  private:
    double _mapParam;
};
} // namespace D5R