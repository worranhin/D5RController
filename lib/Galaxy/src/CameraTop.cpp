/**
 * @file CameraTop.cpp
 * @author worranhin (worranhin@foxmail.com)
 * @author drawal (2581478521@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "CameraTop.h"

namespace D5R {
/**
 * @brief 构造函数
 *
 * @param id 相机的Mac地址
 */
CameraTop::CameraTop(std::string id) : GxCamera(id) {
    // 夹钳模板
    std::string root(ROOT_DIR);
    _clamp.img = cv::imread(root + "/lib/Galaxy/image/model/clamp.png", 0);
    _clamp.center = cv::Point2f(448, 63);
    _clamp.point = cv::Point2f(445.8, 101);
    cv::FileStorage fs1(root + "/lib/Galaxy/image/yml/KeyPoints_Clamp.yml", cv::FileStorage::READ);
    fs1["keypoints"] >> _clamp.keypoints;
    fs1.release();
    cv::FileStorage fs2(root + "/lib/Galaxy/image/yml/Descriptors_Clamp.yml", cv::FileStorage::READ);
    fs2["descriptors"] >> _clamp.descriptors;
    fs2.release();
    // 钳口模板
    _jaw.img = cv::imread(root + "/lib/Galaxy/image/model/jaw_model.png", 0);
    _jaw.center = cv::Point2f(318, 408.5);
    _jaw.point = cv::Point2f(318, 44);
    _jaw.jaw_Circle_Center = cv::Point2f(318.752, 99.4593);
    cv::FileStorage fs3(root + "/lib/Galaxy/image/yml/KeyPoints_Jaw.yml", cv::FileStorage::READ);
    fs3["keypoints"] >> _jaw.keypoints;
    fs3.release();
    cv::FileStorage fs4(root + "/lib/Galaxy/image/yml/Descriptors_Jaw.yml", cv::FileStorage::READ);
    fs4["descriptors"] >> _jaw.descriptors;
    fs4.release();

    _mapParam = 0.00945084;
}

/**
 * @brief 相机析构函数
 *
 */
CameraTop::~CameraTop() {}

/**
 * @brief 获取钳口模板，记录钳口模板的特征信息，
 * 选择哪个模板，相机与机器人就挪到哪个模板对应的位置
 * 该函数已荒废，仅供参考
 *
 * @param img 相机获取的图片，为灰度图
 *
 */
void CameraTop::GetJawModel(cv::Mat img) {
    return;

    if (img.channels() != 1) {
        ERROR_("img should be gray type");
        return;
    }

    cv::Point2f roiP(850, 750);
    cv::Rect roi = cv::Rect(roiP, cv::Size(650, 850));
    cv::Mat roiImg = img(roi).clone();
    _jaw.img = roiImg.clone();
    // SIFT特征提取
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    sift->detect(_jaw.img, _jaw.keypoints);
    sift->compute(_jaw.img, _jaw.keypoints, _jaw.descriptors);

    // 图像处理
    cv::Mat jaw_binary;
    cv::threshold(roiImg, jaw_binary, 101, 255, cv::THRESH_BINARY);
    cv::Mat jaw_Gauss;
    cv::GaussianBlur(jaw_binary, jaw_Gauss, cv::Size(7, 7), 0, 0, cv::BORDER_DEFAULT);
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

    int shortindex = (rect.size.width < rect.size.height) ? 1 : 0;
    cv::Point2f midPoint_up =
        0.5 * (rectPoints[shortindex] + rectPoints[(shortindex + 1) % 4]);

    _jaw.center = rect.center;
    _jaw.point = midPoint_up;
}

/**
 * @brief 获取钳口圆心坐标，仅作参考使用
 *
 * @param img
 */
void CameraTop::GetJawCircleCenter(cv::Mat img) {
    return;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bin;
    cv::threshold(gray, bin, 60, 255, cv::THRESH_BINARY);
    cv::Rect roi(220, 60, 200, 80);
    cv::Mat black = cv::Mat(img.size(), gray.type(), cv::Scalar::all(0));
    bin(roi).copyTo(black(roi));

    cv::Mat inv_bin;
    cv::bitwise_not(black, inv_bin);

    cv::Mat edge;
    cv::Canny(inv_bin, edge, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edge, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> contours_;
    for (auto &contour : contours) {
        contours_.insert(contours_.end(), contour.begin(), contour.end());
    }
    cv::Point2f center;
    float r;
    cv::minEnclosingCircle(contours_, center, r);
    _jaw.jaw_Circle_Center = center;
}

/**
 * @brief 特征匹配算法，image与模板进行匹配，返回模板在image中的位置信息，需要注意的是，如果对JAW进行模板匹配，需确定已经获取JAW模板
 *
 * @param image 相机实时图像，灰度图
 * @param modelname JAW-钳口 CLAMP-夹钳
 * @param pst 模板在像素坐标系下的位置信息，pst[0]:center;pst[1]:positon point
 *
 * @todo 需添加模板匹配不成功时的解决方案
 *
 */
bool D5R::CameraTop::SIFT(cv::Mat image, ModelType modelname,
                          std::vector<cv::Point2f> &pst) {
    cv::Mat model;
    std::vector<cv::Point2f> modelPosition;
    std::vector<cv::KeyPoint> keyPoints_Model;
    cv::Mat descriptors_model;
    if (modelname == ModelType::CLAMP) {
        model = _clamp.img.clone();
        modelPosition.push_back(_clamp.center);
        modelPosition.push_back(_clamp.point);
        keyPoints_Model = _clamp.keypoints;
        descriptors_model = _clamp.descriptors;
    } else {
        model = _jaw.img.clone();
        modelPosition.push_back(_jaw.center);
        modelPosition.push_back(_jaw.point);
        keyPoints_Model = _jaw.keypoints;
        descriptors_model = _jaw.descriptors;
    }

    // ROI
    cv::Point2f roiP(450, 700);
    cv::Rect roi = cv::Rect(roiP, cv::Size(750, 1348));
    cv::Mat ROI = image(roi).clone();

    // SIFT特征点
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keyPoints_Img;
    sift->detect(ROI, keyPoints_Img);
    // 描述
    cv::Mat descriptors_Img;
    sift->compute(ROI, keyPoints_Img, descriptors_Img);
    // 匹配
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> goodMatches;
    matcher->knnMatch(descriptors_model, descriptors_Img, knn_matches, 2);
    for (auto &knn_matche : knn_matches) {
        if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance) {
            goodMatches.push_back(knn_matche[0]);
        }
    }

    // 绘制匹配图
    //  cv::Mat img_matches_knn;
    //  cv::drawMatches(model, keyPoints_Model, ROI, keyPoints_Img, goodMatches, img_matches_knn, cv::Scalar::all(-1),
    //                  cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //  cv::imwrite("E:/WYL_workspace/D5RC/image/11_22/res_match_123141.png", img_matches_knn);

    // 计算
    std::vector<cv::Point2f> model_P, img_P;
    for (const auto &match : goodMatches) {
        model_P.push_back(keyPoints_Model[match.queryIdx].pt);
        img_P.push_back(keyPoints_Img[match.trainIdx].pt);
    }

    if (img_P.size() < 8) {
        return false;
    }

    cv::Mat homography = cv::findHomography(model_P, img_P, cv::RANSAC);

    cv::perspectiveTransform(modelPosition, pst, homography);
    return true;
    // pst[0] += roiP;
    // pst[1] += roiP;
}

/**
 * @brief 获取相机映射参数
 *
 * @param img 标定图片
 */
void CameraTop::GetMapParam(cv::Mat img) {
    std::vector<cv::Point2f> corner;
    if (!cv::findChessboardCorners(img, cv::Size(19, 15), corner)) {
        std::cerr << "Failed to find corners in image" << std::endl;
        return;
    }
    const cv::TermCriteria criteria{cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 0.001};
    cv::cornerSubPix(img, corner, cv::Size(6, 6), cv::Size(-1, -1), criteria);
    cv::drawChessboardCorners(img, cv::Size(19, 15), corner, true);

    std::string imagename = "Calibration_board";
    cv::namedWindow(imagename, cv::WINDOW_NORMAL);
    cv::resizeWindow(imagename, cv::Size(1295, 1024));
    cv::imshow(imagename, img);

    float sum = 0;
    cv::Point2f last_point{};
    int count = 0;
    for (cv::Point2f point : corner) {
        count++;
        if (count != 1 && (count - 1) % 19 != 0) {
            auto a = sqrt(powf(point.x - last_point.x, 2) + powf(point.y - last_point.y, 2));
            sum += a;
        }
        last_point = point;
    }
    float map_param = 18 * 15 / sum;
    std::cout << "map_param: " << map_param << " (mm/pixel)" << std::endl;
    cv::waitKey(0);
}

/**
 * @brief 读取相机图片，返回夹钳、钳口在像素坐标的位姿信息
 *
 * @return std::vector<std::vector<float>> 返回参数列表： {{jaw.center.x,
 * jaw.center.y, jaw_angle}, {clamp.center.x, clamp.center.y, clamp_angle}}
 */
std::vector<std::vector<float>> CameraTop::GetPixelPos() {
    cv::Point2f roiP(800, 648);
    std::vector<std::vector<float>> pos;
    // Read(_img);
    // if (_img.empty()) {
    //     std::cerr << "Failed to read img" << std::endl;
    //     return pos;
    // }
    // int count = 0;
    cv::Mat img;
    Read(img);
    std::vector<cv::Point2f> pos_jaw;
    if (!SIFT(img, JAW, pos_jaw)) {
        Read(img);
        SIFT(img, JAW, pos_jaw);
    }
    float angle_jaw =
        atan2f(pos_jaw[1].y - pos_jaw[0].y, pos_jaw[1].x - pos_jaw[0].x) * (-180) / CV_PI;
    pos.push_back({pos_jaw[0].x, pos_jaw[0].y, angle_jaw});
    std::vector<cv::Point2f> pos_clamp;

    if (!SIFT(img, CLAMP, pos_clamp)) {
        Read(img);
        SIFT(img, CLAMP, pos_clamp);
    }
    float angle_clamp =
        atan2f(pos_clamp[0].y - pos_clamp[1].y, pos_clamp[0].x - pos_clamp[1].x) * (-180) / CV_PI;
    pos.push_back({pos_clamp[0].x, pos_clamp[0].y, angle_clamp});

    cv::line(img, pos_jaw[0] + roiP, pos_jaw[1] + roiP, cv::Scalar(0), 4);
    cv::line(img, pos_clamp[0] + roiP, pos_clamp[1] + roiP, cv::Scalar(0), 4);
    cv::putText(img, std::to_string(angle_jaw), pos_jaw[1] + roiP,
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 4);
    cv::putText(img, std::to_string(angle_clamp), pos_clamp[0] + roiP,
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 4);
    std::string windowname = "image";
    cv::namedWindow(windowname, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowname, cv::Size(1295, 1024));
    cv::imshow(windowname, img);
    cv::waitKey(0);
    return pos;
}

/**
 * @brief 返回物理坐标系下的位置信息
 *
 * @return std::vector<std::vector<double>> 返回参数列表：
 * {{jaw.center.x, jaw.center.y, jaw_angle}, {clamp.center.x, clamp.center.y,
 * clamp_angle}}
 */
std::vector<std::vector<double>> CameraTop::GetPhysicPos() {
    auto pixelPos = GetPixelPos();
    std::vector<std::vector<double>> physicPos;
    for (auto &p : pixelPos) {
        physicPos.push_back({p[0] * _mapParam, p[1] * _mapParam, p[2]});
    }
    return physicPos;
}

/**
 * @brief 计算夹钳和钳口在物理坐标系下的位置差
 *
 * @return std::vector<double> 返回参数列表：
 * {jaw.center.x - clamp.center.x, jaw.center.y - clamp.center.y, jaw_angle -
 * clamp_angle}
 */
std::vector<double> CameraTop::GetPhysicError() {
    auto pixelPos = GetPixelPos();
    std::vector<double> posError;
    posError.push_back((pixelPos[0][0] - pixelPos[1][0]) * _mapParam);
    posError.push_back((pixelPos[0][1] - pixelPos[1][1]) * _mapParam);
    posError.push_back(pixelPos[0][2] - pixelPos[1][2]);
    return posError;
}

} // namespace D5R