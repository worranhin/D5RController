#pragma once
#include "LogUtil.h"
#include "VCSDK/inc/GxIAPI.h"
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
  bool Init();
  bool IsInit();
  bool Reconnect();
  void Release();
  bool Read(cv::OutputArray image);
  bool Retrieve(cv::OutputArray image);

private:
  GX_DEV_HANDLE _handle{};
  std::string _id{};
  bool _isInit{};
  GX_FRAME_DATA _data{};
  int64_t _payload{};
};
} // namespace D5R
