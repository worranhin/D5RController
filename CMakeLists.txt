# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义项目名称和使用的编程语言
project(D5RC VERSION 0.2)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(lib/RMD)
add_subdirectory(lib/Nator)
add_subdirectory(lib/Galaxy)

add_library(D5Robot src/D5Robot.cpp)
target_include_directories(D5Robot PUBLIC "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/lib/RMD" "${PROJECT_SOURCE_DIR}/lib/Nator" "${PROJECT_SOURCE_DIR}/lib/Galaxy")
target_link_libraries(D5Robot PUBLIC RMDMotor)
target_link_libraries(D5Robot PUBLIC NatorMotor)
target_link_libraries(D5Robot PUBLIC Camera)

add_library(D5RbotApi SHARED src/DllApi.cpp)
target_include_directories(D5RbotApi PUBLIC "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(D5RbotApi D5Robot)
target_compile_definitions(D5RbotApi PUBLIC "D5R_EXPORTS")

add_executable(test test.cpp)
target_link_libraries(test D5Robot)
target_link_libraries(test D5RbotApi)

add_executable(TestKineHelper "test/TestKineHelper.cpp")
target_include_directories(TestKineHelper PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(TestKineHelper D5Robot)