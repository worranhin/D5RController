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
find_package(OpenCV REQUIRED)  # 搜索系统中的 OpenCV，若要自行设定请修改上面的代码并取消注释
include_directories(${OpenCV_INCLUDE_DIR})
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found, please add OpenCV to path or set OpenCV_DIR.")
endif()

# GalaxyCamera 基类
add_library(GalaxyCamera GalaxyCamera.cpp)
target_link_libraries(GalaxyCamera GxAPI)
target_include_directories(GalaxyCamera PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/OpenCV_4_10/include" "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/inc")
target_link_libraries(GalaxyCamera ${OpenCV_LIBS})

# 上方相机类
add_library(CameraUp CameraUP.cpp)
target_link_libraries(CameraUp GalaxyCamera)

# 测试代码用的可执行文件
add_executable(Main main.cpp)
target_link_libraries(Main CameraUp)
target_link_libraries(Main GxAPI)
# target_link_libraries(Main DxImageProc)
target_link_libraries(Main ${OpenCV_LIBS})
# target_link_libraries(Main GalaxyCamera)
target_include_directories(Main PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/inc")

# add_executable(Test test.cpp)
# target_link_libraries(Test GxAPI)
# target_link_libraries(Test CameraUp)
# target_link_libraries(Test GalaxyCamera)
# target_include_directories(Test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/lib/VCSDK/inc")

# TODO: 下面的代码暂时保留，测试后删除 
# add_library(GalaxySDK SHARED IMPORTED)
# set_target_properties(GalaxySDK PROPERTIES
#     IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/VCSDK/dll/GxIAPI.dll"
# )
# set_target_properties(GalaxySDK PROPERTIES
#     IMPORTED_IMPLIB
#     "${PROJECT_SOURCE_DIR}/VCSDK/lib/x64/GxIAPI.lib"
# )


# list(APPEND CMAKE_PREFIX_PATH "../OpenCV_4_10/x64/mingw/bin")
# set(OpenCV_DIR "../OpenCV_4_10")
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIR})

# if(OpenCV_FOUND)
#     target_link_libraries(Galaxy ${OpenCV_LIBS})
# endif()

# add_library(Camera CameraUP.cpp)
# target_link_libraries(Camera Galaxy)
