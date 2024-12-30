# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 定义项目名称和使用的编程语言
project(D5RC VERSION 0.2)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(lib/RMD)
add_subdirectory(lib/Nator)

add_library(D5Robot "src/D5Robot.cpp")
target_include_directories(D5Robot PUBLIC "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/lib/RMD" "${PROJECT_SOURCE_DIR}/lib/Nator")
target_link_libraries(D5Robot PUBLIC RMDMotor)
target_link_libraries(D5Robot PUBLIC NatorMotor)

# 下面将 Cpp Class 转写为动态库
add_library(D5RobotDll SHARED "src/D5RobotDll.cpp")
target_include_directories(D5RobotDll PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(D5RobotDll PUBLIC D5Robot)
target_compile_definitions(D5RobotDll PUBLIC "D5R_EXPORTS")

add_executable(test test.cpp)
target_link_libraries(test D5Robot)
target_link_libraries(test D5RobotDll)



# 包含头文件目录
# include_directories(include)

# 将子目录下的CMakeLists.txt文件包括到当前的构建中

# 链接库 SHARED 共享库
# add_library(DOF5RobotControl SHARED DOF5RobotControl.cpp)

# 设置包含目录  确保编译时可以找到头文件
# target_include_directories(DOF5RobotControl PUBLIC "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/lib/RMDControl" "${PROJECT_SOURCE_DIR}/lib/NatorControl")

# target_link_libraries(DOF5RobotControl PUBLIC RMDControl)
# target_link_libraries(DOF5RobotControl PUBLIC NatorControl)

# 为指定目标添加预处理宏定义
# target_compile_definitions(DOF5RobotControl PRIVATE "D5R_EXPORTS")

# 定义可执行文件及其源文件
# add_executable(D5R_Main main.cpp)

# 链接库到可执行文件
# target_include_directories(D5R_Main PUBLIC "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/lib/RMDControl" "${PROJECT_SOURCE_DIR}/lib/NatorControl")

# target_link_libraries(D5R_Main PUBLIC RMDControl)
# target_link_libraries(D5R_Main PUBLIC NatorControl)
# target_link_libraries(D5R_Main PUBLIC DOF5RobotControl)

# 启用测试
# enable_testing()

# 定义测试用例
# add_test(NAME MyTest COMMAND MyExecutable)

# 设置安装规则
# install(TARGETS MyExecutable RUNTIME DESTINATION bin)
# install(FILES datafile.txt DESTINATION share/data)

# 设置包信息
# set(CPACK_PACKAGE_NAME "DOF5RobotControl")
# set(CPACK_PACKAGE_VERSION "${MyProject_VERSION}")
# set(CPACK_PACKAGE_CONTACT "Hin <worranhin@foxmail.com>")
# include(CPack)

# enable_testing()