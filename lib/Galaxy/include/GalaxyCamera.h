/**
 * @file GalaxyCamera.h
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief GxCamera Class
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "RobotException.hpp"
#include <GxIAPI.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace D5R {

class GxCamera {
  public:
    GxCamera(std::string_view id);
    ~GxCamera();
    const char *GetGxError();
    void Init();
    void Reconnect();
    void Release();
    bool Read(cv::OutputArray image);
    bool Retrieve(cv::OutputArray image);

  private:
    GX_DEV_HANDLE _handle{};
    std::string _id{};
    GX_FRAME_DATA _data{};
    int64_t _payload{};
    cv::Mat _map1{};
    cv::Mat _map2{};
};
} // namespace D5R
