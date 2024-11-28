#pragma once
#include "GalaxyCamera.h"

namespace D5R {

struct Model {
    cv::Mat img;
    cv::Point2f center;
    cv::Point2f point;
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
    bool SIFT(cv::Mat img, ModelType modelname, std::vector<cv::Point2f> &pst);
    void GetMapParam(cv::Mat Calibration_board);
    std::vector<std::vector<float>> GetPixelPos();
    std::vector<std::vector<double>> GetPhysicPos();
    std::vector<double> GetPhysicError();

  private:
    Model _jaw;
    Model _clamp; // 手动初始化
    cv::Mat _img;
    double _mapParam;
};

} // namespace D5R