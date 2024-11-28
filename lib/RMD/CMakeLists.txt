cmake_minimum_required(VERSION 3.10) # 设置CMake的最低版本需求

project(RMD VERSION 0.2) # 定义项目名称

# 设置C标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(SerialPort SerialPort.cpp)
target_include_directories(SerialPort PUBLIC "${CMAKE_SOURCE_DIR}/global")

add_library(RMDMotor RMDMotor.cpp)
target_link_libraries(RMDMotor SerialPort)
