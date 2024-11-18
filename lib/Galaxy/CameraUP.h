#pragma once
#include "GalaxyCamera.h"

namespace D5R {

struct Model {
  cv::Mat img;
  cv::Point2f center;
  cv::Point2f point;
};

enum ModelType { JAW = 0, CLAMP = 1 };

class CameraUP : public GxCamera {
public:
  CameraUP(std::string_view id);
  ~CameraUP();
  void GetJawModel(cv::Mat img, std::vector<cv::Point2f> &pst);
  void SIFT(cv::Mat img, ModelType modelname, std::vector<cv::Point2f> &pst);
  std::vector<std::vector<float>> GetPos();

private:
  Model _jaw;
  Model _clamp; // 手动初始化
};

} // namespace D5R