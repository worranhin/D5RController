cmake_minimum_required(VERSION 3.10)
project(VisionModule VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 导入 DLL 库
add_library(GxAPI SHARED IMPORTED)
set_property(TARGET GxAPI PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/dll/GxIAPI.dll")
set_property(TARGET GxAPI PROPERTY IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/lib/x64/GxIAPI.lib")

add_library(DxImageProc SHARED IMPORTED)
set_property(TARGET DxImageProc PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/dll/DxImageProc.dll")
set_property(TARGET DxImageProc PROPERTY IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/lib/x64/DxImageProc.lib")

# 导入 OpenCV 模块
# list(APPEND CMAKE_PREFIX_PATH "../OpenCV_4_10/x64/mingw/bin")
# set(OpenCV_DIR "D:/Documents/SCUT/D5RController/lib/OpenCV_4_10")
find_package(OpenCV REQUIRED) # 搜索系统中的 OpenCV，若要自行设定请修改上面的代码并取消注释
include_directories(${OpenCV_INCLUDE_DIR})

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found, please add OpenCV to path or set OpenCV_DIR.")
endif()

# GalaxyCamera 基类
add_library(GalaxyCamera src/GalaxyCamera.cpp)
target_include_directories(GalaxyCamera PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/OpenCV_4_10/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/inc"
    "${CMAKE_SOURCE_DIR}/global"
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(GalaxyCamera GxAPI)
target_link_libraries(GalaxyCamera ${OpenCV_LIBS})

# 上方相机类
add_library(CameraTop src/CameraTop.cpp)
target_link_libraries(CameraTop GalaxyCamera)
