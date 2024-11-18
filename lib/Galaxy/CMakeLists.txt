cmake_minimum_required(VERSION 3.10)
project(GalaxySDK VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(GalaxySDK SHARED IMPORTED)

set_target_properties(GalaxySDK PROPERTIES
    IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/VCSDK/dll/GxIAPI.dll"
)
set_target_properties(GalaxySDK PROPERTIES
    IMPORTED_IMPLIB
    "${PROJECT_SOURCE_DIR}/VCSDK/lib/x64/GxIAPI.lib"
)

add_library(Galaxy GalaxyCamera.cpp)
target_link_libraries(Galaxy GalaxySDK)
target_include_directories(Galaxy PUBLIC "../../include")

list(APPEND CMAKE_PREFIX_PATH "../OpenCV_4_10/x64/mingw/bin")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

if(OpenCV_FOUND)
    target_link_libraries(Galaxy ${OpenCV_LIBS})
endif()

add_library(Camera CameraUP.cpp)
target_link_libraries(Camera Galaxy)

# add_executable(Test test.cpp)
# target_link_libraries(Test CameraUP)