/**
 * @file CameraTop.h
 * @author worranhin (worranhin@foxmail.com)
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

struct Model {
    cv::Mat img;
    cv::Point2f center;
    cv::Point2f point;
    cv::Point2f jaw_Circle_Center;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

enum ModelType { JAW = 0,
                 CLAMP = 1 };

class CameraTop : public GxCamera {
  public:
    CameraTop(std::string id);
    ~CameraTop();
    void GetJawModel(cv::Mat img);
    void GetJawCircleCenter(cv::Mat img);
    bool SIFT(cv::Mat img, ModelType modelname, std::vector<cv::Point2f> &pst);
    void GetMapParam(cv::Mat Calibration_board);
    std::vector<std::vector<float>> GetPixelPos();
    std::vector<std::vector<double>> GetPhysicPos();
    std::vector<double> GetPhysicError();

  private:
    Model _jaw;
    Model _clamp;
    cv::Mat _img;
    double _mapParam;
};

} // namespace D5R