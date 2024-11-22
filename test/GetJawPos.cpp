#include "GalaxyCamera.h"

int main() {
    cv::Mat img = cv::imread("../image/11_22/jaw0.png", 0);
    if (img.channels() != 1) {
        std::cout << "kahui" << std::endl;
        return -1;
    }

    cv::Mat bulr;
    cv::medianBlur(img, bulr, 5);

    cv::Point2f roiP(860, 720);
    cv::Rect roi = cv::Rect(roiP, cv::Size(620, 780));
    cv::Mat roiImg = bulr(roi).clone();
    cv::imwrite("../image/11_22/jaw_model.png", roiImg);

    // 图像处理
    cv::Mat jaw_binary;
    cv::threshold(roiImg, jaw_binary, 60, 255, cv::THRESH_BINARY);
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

    cv::Mat black = cv::Mat(roiImg.size(), roiImg.type(), cv::Scalar::all(0));

    if (hull.size() > 1) {
        for (int i = 0; i < hull.size() - 1; i++) {
            cv::line(black, hull[i], hull[i + 1], cv::Scalar(255), 2);
        }
        cv::line(black, hull[hull.size() - 1], hull[0], cv::Scalar(255), 2);
    }
    // 获取最小外接矩形
    cv::RotatedRect rect = cv::minAreaRect(hull);
    cv::Point2f rectPoints[4];
    rect.points(rectPoints);

    int shortindex = (rect.size.width < rect.size.height) ? 1 : 0;
    cv::Point2f midPoint_up =
        0.5 * (rectPoints[shortindex] + rectPoints[(shortindex + 1) % 4]);

    cv::line(roiImg, midPoint_up, rect.center, cv::Scalar(0), 2);
    for (int i = 0; i < 4; i++) {
        cv::line(roiImg, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(0), 2);
    }
    std::cout << rect.center << std::endl;
    std::cout << midPoint_up << std::endl;

    cv::imshow("sdf", edges);
    cv::waitKey(0);
    cv::imshow("sdf", black);
    cv::waitKey(0);
    cv::imshow("sdf", roiImg);
    cv::waitKey(0);

    return 0;
}