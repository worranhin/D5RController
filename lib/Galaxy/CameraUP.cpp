#include "CameraUP.h"

namespace D5R {

CameraUP::CameraUP(std::string_view id) : GxCamera(id) {
  _clamp.img = cv::imread("./image/model/clamp.png", 0);
  _clamp.center = cv::Point2f(448, 63);
  _clamp.point = cv::Point2f(445.8, 101);
}
CameraUP::~CameraUP() {}
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

} // namespace D5R