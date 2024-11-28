/**
 * @file CameraBot.cpp
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "CameraBot.h"

namespace D5R {

CameraBot::CameraBot(std::string id) : GxCamera(id) {
    _mapParam = _mapParam = 0.00945084;
}

CameraBot::~CameraBot() {}
} // namespace D5R