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

    std::vector<float> GetHorizontalLine(cv::Mat img);
    std::vector<cv::Point2f> GetModelPoints(cv::Mat img);
    double GetDistance(cv::Mat img, std::vector<float> line_params);

  private:
    double _mapParam;
    cv::Mat _clamp;
    std::vector<cv::Point2f> _points;
};
} // namespace D5R