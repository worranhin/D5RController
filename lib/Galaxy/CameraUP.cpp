#include "CameraUP.h"

namespace D5R {
/**
 * @brief 构造函数
 *
 * @param id 相机的Mac地址
 */
CameraUP::CameraUP(std::string_view id) : GxCamera(id) {
  _clamp.img = cv::imread("./image/model/clamp.png", 0);
  _clamp.center = cv::Point2f(448, 63);
  _clamp.point = cv::Point2f(445.8, 101);
  _mapParam = 0.00945188;
}

/**
 * @brief 相机析构函数
 *
 */
CameraUP::~CameraUP() {}

/**
 * @brief
 * 获取钳口模板，记录钳口模板的定位信息，并返回钳口的初始位置信息，用以初始化夹钳位置
 *
 * @param img 相机获取的图片，为灰度图
 * @param pst 钳口在像素坐标系下的位置信息，pst[0]:center;pst[1]:up positon
 * point
 */
void CameraUP::GetJawModel(cv::Mat img, std::vector<cv::Point2f> &pst) {
  if (img.channels() != 1) {
    ERROR_("img should be gray type");
    return;
  }
  // ROI区域--后续改成识别头顶的方块
  cv::Point2f p0(950, 430);
  cv::Rect roi(p0.x, p0.y, 650, 750);
  cv::Mat roiImg = img(roi);
  _jaw.img = roiImg.clone();
  cv::Mat jaw = cv::Mat(img.size(), img.type(), cv::Scalar::all(255));
  roiImg.copyTo(jaw(roi));
  // 图像处理
  cv::Mat jaw_binary;
  cv::threshold(jaw, jaw_binary, 101, 255, cv::THRESH_BINARY);
  cv::Mat jaw_Gauss;
  cv::GaussianBlur(jaw_binary, jaw_Gauss, cv::Size(7, 7), 0, 0,
                   cv::BORDER_DEFAULT);
  // Canny边缘检测
  cv::Mat edges;
  cv::Canny(jaw_Gauss, edges, 50, 150);
  // 轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // 压缩
  std::vector<cv::Point> contours_1;
  for (const auto &contour : contours) {
    // 丢弃小的轮廓
    if (contour.size() < 200) {
      continue;
    }
    contours_1.insert(contours_1.end(), contour.begin(), contour.end());
  }

  // 凸包
  std::vector<cv::Point> hull;
  cv::convexHull(cv::Mat(contours_1), hull);
  // 获取最小外接矩形
  cv::RotatedRect rect = cv::minAreaRect(hull);
  cv::Point2f rectPoints[4];
  rect.points(rectPoints);
  pst.push_back(rect.center);

  int shortindex = (rect.size.width < rect.size.height) ? 1 : 0;
  cv::Point2f midPoint_up =
      0.5 * (rectPoints[shortindex] + rectPoints[(shortindex + 1) % 4]);
  pst.push_back(midPoint_up);

  _jaw.center = rect.center - p0;
  _jaw.point = midPoint_up - p0;
}

/**
 * @brief
 * 特征匹配算法，image与模板进行匹配，返回模板在image中的位置信息，需要注意的是，如果对JAW进行模板匹配，需确定已经获取JAW模板
 *
 * @param image 相机实时图像，灰度图
 * @param modelname JAW-钳口 CLAMP-夹钳
 * @param pst 模板在像素坐标系下的位置信息，pst[0]:center;pst[1]:positon point
 *
 */
void D5R::CameraUP::SIFT(cv::Mat image, ModelType modelname,
                         std::vector<cv::Point2f> &pst) {
  cv::Mat model;
  std::vector<cv::Point2f> modelPosition;
  if (modelname == ModelType::CLAMP) {
    model = _clamp.img.clone();
    modelPosition.push_back(_clamp.center);
    modelPosition.push_back(_clamp.point);
  } else {
    model = _jaw.img.clone();
    modelPosition.push_back(_jaw.center);
    modelPosition.push_back(_jaw.point);
  }
  // SIFT特征点
  cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
  std::vector<cv::KeyPoint> keyPoints_Model;
  sift->detect(model, keyPoints_Model);
  std::vector<cv::KeyPoint> keyPoints_Img;
  sift->detect(image, keyPoints_Img);
  // 描述
  cv::Mat descriptors_model;
  sift->compute(model, keyPoints_Model, descriptors_model);
  cv::Mat descriptors_Img;
  sift->compute(image, keyPoints_Img, descriptors_Img);
  // 匹配
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  const float ratio_thresh = 0.75f;
  std::vector<cv::DMatch> goodMatches;
  matcher->knnMatch(descriptors_model, descriptors_Img, knn_matches, 2);
  for (auto &knn_matche : knn_matches) {
    if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance) {
      goodMatches.push_back(knn_matche[0]);
    }
  }
  // 计算
  std::vector<cv::Point2f> model_P, img_P;
  for (const auto &match : goodMatches) {
    model_P.push_back(keyPoints_Model[match.queryIdx].pt);
    img_P.push_back(keyPoints_Img[match.trainIdx].pt);
  }
  cv::Mat homography = cv::findHomography(model_P, img_P, cv::RANSAC);
  cv::perspectiveTransform(modelPosition, pst, homography);
}

/**
 * @brief 返回相机映射参数
 *
 * @return double 映射参数
 */
double CameraUP::GetMapParam() { return _mapParam; }

/**
 * @brief 读取相机图片，返回夹钳、钳口在像素坐标的位姿信息
 *
 * @return std::vector<std::vector<float>> 返回参数列表： {{jaw.center.x,
 * jaw.center.y, jaw_angle}, {clamp.center.x, clamp.center.y, clamp_angle}}
 */
std::vector<std::vector<float>> CameraUP::GetPixelPos() {
  std::vector<std::vector<float>> pos;
  if (_jaw.img.empty()) {
    std::cerr << "Please run GetJawModel fun first" << std::endl;
    return pos;
  }
  Read(_img);
  if (_img.empty()) {
    std::cerr << "Failed to read img" << std::endl;
    return pos;
  }
  cv::Mat img = _img.clone();
  std::vector<cv::Point2f> pos_jaw;
  SIFT(img, JAW, pos_jaw);
  float angle_jaw =
      atan2f(pos_jaw[1].y - pos_jaw[0].y, pos_jaw[1].x - pos_jaw[0].x) *
      (-180) / CV_PI;
  pos.push_back({pos_jaw[0].x, pos_jaw[0].y, angle_jaw});
  std::vector<cv::Point2f> pos_clamp;
  SIFT(img, CLAMP, pos_clamp);
  float angle_clamp =
      atan2f(pos_clamp[0].y - pos_clamp[1].y, pos_clamp[0].x - pos_clamp[1].x) *
      (-180) / CV_PI;
  pos.push_back({pos_clamp[0].x, pos_clamp[0].y, angle_jaw});
  cv::line(img, pos_jaw[0], pos_jaw[1], cv::Scalar(0), 2);
  cv::line(img, pos_clamp[0], pos_clamp[1], cv::Scalar(0), 2);
  cv::putText(img, std::to_string(angle_jaw), pos_jaw[1],
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0));
  cv::putText(img, std::to_string(angle_clamp), pos_clamp[0],
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0));
  std::string windowname = "image";
  cv::namedWindow(windowname, cv::WINDOW_NORMAL);
  cv::resizeWindow(windowname, cv::Size(1295, 1024));
  cv::imshow(windowname, img);
  return pos;
}

/**
 * @brief 返回物理坐标系下的位置信息
 *
 * @return std::vector<std::vector<double>> 返回参数列表：
 * {{jaw.center.x, jaw.center.y, jaw_angle}, {clamp.center.x, clamp.center.y,
 * clamp_angle}}
 */
std::vector<std::vector<double>> CameraUP::GetPhysicPos() {
  auto pixelPos = GetPixelPos();
  std::vector<std::vector<double>> physicPos;
  for (auto &p : pixelPos) {
    physicPos.push_back({p[0] * _mapParam, p[1] * _mapParam, p[2] * _mapParam});
  }
  return physicPos;
}

} // namespace D5R