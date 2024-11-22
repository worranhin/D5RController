#include "GalaxyCamera.h"

int main() {
    cv::Mat model = cv::imread("../image/11_22/jaw_model.png", 0);
    cv::Mat image = cv::imread("../image/11_22/jaw0.png");
    if (image.empty()) {
        std::cerr << "Failed to load model " << std::endl;
        return -1;
    }
    cv::Rect roi = cv::Rect(800, 648, 750, 1400);
    cv::Mat ROI = image(roi).clone();

    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();

    // std::vector<cv::KeyPoint> keyPoints_Model;
    // sift->detect(model, keyPoints_Model);
    // cv::FileStorage fs1("../test/yml/KeyPoints_Jaw.yml", cv::FileStorage::WRITE);
    // fs1 << "keypoints" << keyPoints_Model;
    // fs1.release();

    // cv::Mat descriptors_model;
    // sift->compute(model, keyPoints_Model, descriptors_model);
    // cv::FileStorage fs2("../test/yml/Descriptors_Jaw.yml", cv::FileStorage::WRITE);
    // fs2 << "descriptors" << descriptors_model;
    // fs2.release();

    std::vector<cv::KeyPoint> keyPoints_Model;
    cv::FileStorage fs1("../test/yml/KeyPoints_Jaw.yml", cv::FileStorage::READ);
    fs1["keypoints"] >> keyPoints_Model;
    fs1.release();

    cv::Mat descriptors_model;
    cv::FileStorage fs2("../test/yml/Descriptors_Jaw.yml", cv::FileStorage::READ);
    fs2["descriptors"] >> descriptors_model;
    fs2.release();

    int64 start = cv::getTickCount();
    std::vector<cv::KeyPoint> keyPoints_Img;
    sift->detect(ROI, keyPoints_Img);
    cv::Mat descriptors_Img;
    sift->compute(ROI, keyPoints_Img, descriptors_Img);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> goodMatches;
    matcher->knnMatch(descriptors_model, descriptors_Img, knn_matches, 2);
    for (auto &knn_matche : knn_matches) {
        if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance) {
            goodMatches.push_back(knn_matche[0]);
        }
    }

    std::vector<cv::Point2f> model_P, img_P;
    for (const auto &match : goodMatches) {
        model_P.push_back(keyPoints_Model[match.queryIdx].pt);
        img_P.push_back(keyPoints_Img[match.trainIdx].pt);
    }
    cv::Mat homography = cv::findHomography(model_P, img_P, cv::RANSAC);

    std::vector<cv::Point2f> modelPosition = {cv::Point2f(318, 408.5), cv::Point2f(318, 44)};
    std::vector<cv::Point2f> ImgPosition;
    cv::perspectiveTransform(modelPosition, ImgPosition, homography);
    cv::line(image, ImgPosition[0] + cv::Point2f(800, 648), ImgPosition[1] + cv::Point2f(800, 648), cv::Scalar(0, 0, 255), 2);
    float angle = atan2f((ImgPosition[0].y - ImgPosition[1].y), (ImgPosition[0].x - ImgPosition[1].x)) * (-180) / CV_PI;
    cv::putText(image, std::to_string(angle), ImgPosition[0] + cv::Point2f(800, 648), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);

    int64 end = cv::getTickCount();
    std::cout << 1000.0 * (end - start) / cv::getTickFrequency() << std::endl;

    cv::Mat img_matches_knn;
    cv::drawMatches(model, keyPoints_Model, ROI, keyPoints_Img, goodMatches, img_matches_knn, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imwrite("../image/11_22/res_match_knn_0452.png", img_matches_knn);
    std::string windowname2 = "Match res";
    cv::namedWindow(windowname2, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowname2, cv::Size(1295, 1024));
    cv::imshow(windowname2, image);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}