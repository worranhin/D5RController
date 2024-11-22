#include "CameraUP.h"
#include <GxIAPI.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    std::cout << "Hello Galaxy!" << std::endl;

    using namespace D5R;
    CameraUP("1");

    return 0;
}

void testOpenCV() {
    std::string imagePath = "image.jpg";
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    cv::imshow("Image", image);
    cv::waitKey(0);
}

// const char *GxCamera::GetGxError() {
//   size_t size = 256;
//   static char err_info[256];
//   GXGetLastError(nullptr, err_info, &size);
//   return err_info;
// }

int TestGalaxy() {
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    emStatus = GXInitLib(); // 打开库
    if (emStatus != GX_STATUS_SUCCESS) {
        std::cout << "GXInitLib error: " << emStatus << std::endl;
        return -1;
    }

    uint32_t deviceNum = 0;
    emStatus = GXUpdateAllDeviceList(&deviceNum, 1000);
    if (emStatus != GX_STATUS_SUCCESS) {
        std::cout << "GXUpdateAllDeviceList error: " << emStatus << std::endl;
        return -1;
    }

    std::cout << "Found " << deviceNum << " device(s)" << std::endl;

    emStatus = GXCloseLib(); // 关闭库
    if (emStatus != GX_STATUS_SUCCESS) {
        std::cout << "GXCloseLib error: " << emStatus << std::endl;
        return -1;
    }

    return 0;
}