# D5RC

这是用于控制五自由度机器人的 C/C++ 库代码。

> 本仓库是[该仓库](https://github.com/worranhin/DOF5RobotControl)的类实现版本。

## 使用

> 本库仅在 Windows 平台下开发并测试

### 配置 OpenCV

1. 去 OpenCV 官网下载 PreBuilt 包并安装 [[链接在此](https://github.com/opencv/opencv/releases/latest)]
2. 将如下路径添加进环境变量的 Path 中  
    `path\to\opencv\build\x64\vc16\bin`  
    `path\to\opencv\build\x64\vc16\lib`
3. 添加一个名为 `OpenCV_DIR` 的环境变量，将其值设为 `path\to\opencv\build` （或者自行在 `CMakeLists.txt` 中设置应该也是可以的）

### 配置 Nators 电机 SDK

进入 `lib/NatorControl/SDK1.4.12` 按指示操作。

> 为什么？这是厂家的 bug 处理方式。

### 配置相机 SDK

~~注意！需要将lib/Galaxy/VCSDK/dll/GxlAPI.dll加入根目录下的build文件中（问题应该已解决，充分验证后删除此条目）~~

> ~~小编也很无奈，能力不足，无法解决该bug~~

### 配置编译器

将 Cmake 的编译器设置留空，或设置为 MSVC（OpenCV PreBuilt 包不支持 Mingw，若要使用请自行解决）