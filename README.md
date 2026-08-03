# auto_aim_example

RM 装甲板识别示例工程。当前项目从海康工业相机取帧，将图像转换为 OpenCV `cv::Mat`，再完成图像预处理、灯条筛选、灯条颜色识别、装甲板配对和 PnP 位姿解算。

## Pipeline

当前视觉流程由 `vision_pipeline` 里的自由函数串联（入口 `runPipeline`）：

```text
HikCapture
  -> preprocessFrame
  -> filterLightBars
  -> matchArmors
  -> solvePnp
```

处理步骤：

1. 相机取帧并转换为 `cv::Mat`
2. 灰度化
3. 二值化提取亮区
4. 开运算、闭运算
5. `findContours` 提取最外层轮廓
6. `minAreaRect` 得到候选最小外接矩形
7. `fitLine` 得到轮廓中心线
8. 按面积、长短边比、角度、填充率筛选灯条
9. 在原图轮廓 mask 内比较红/蓝通道总和，识别灯条颜色
10. 按长度比、角度差、中心 y 差、中心距比例、无遮挡、同色规则配对装甲板
11. 按 small/large 装甲板尺寸生成 3D 点，用左右灯条 top/bottom 点做 2D 点并调用 `solvePnP`

## 目录结构

  1. 相机采集

  - include/hik_capture.hpp
  - src/hik_capture.cpp

  封装海康工业相机 SDK，负责设备枚举、相机配置、取帧，以及将相机图像转换为 OpenCV 的 cv::Mat。

  2. 装甲板视觉识别流水线

  核心流程是：

  相机图像
    → 图像预处理
    → 灯条筛选
    → 灯条配对
    → PnP 位姿解算

  对应文件：

  - src/armor_preprocessor.cpp：灰度化、二值化、形态学处理和轮廓提取。
  - src/light_bar_filter.cpp：按照面积、长宽比、角度、填充率筛选灯条，并判断红蓝颜色。
  - src/armor_matcher.cpp：将两个同色灯条配对成装甲板，并区分大小装甲板。
  - src/pnp_solver.cpp：结合相机内参计算装甲板的旋转和平移。
  - src/vision_pipeline.cpp：负责串联上述步骤。
  - include/vision_pipeline.hpp：定义公共数据结构、参数和函数接口。

  3. 主程序与调试显示

  - src/main.cpp：程序入口，打开第一台相机、循环取帧、运行识别流水线并输出识别结果。
  - src/debug_draw.cpp：绘制灯条、装甲板和二值图等调试信息。

  4. 相机标定

  - src/calibrate_main.cpp：棋盘格标定程序，可实时采图或读取已有图片。
  - config/camera_calibration.yml：运行时使用的相机内参和畸变参数。
  - tools/convert_calibration_npy.js：把 Python 的 .npy 标定结果转为 OpenCV YAML。

  5. 模型和数据资源

  - model/mlp.onnx
  - model/label.txt

  仓库中包含一个 ONNX MLP 模型和标签文件，但当前 CMake 和视觉流水线没有加载它们，所以它们目前没有参与实际识别。

  6. 构建、交叉编译与部署

  - CMakeLists.txt：构建主程序 hik_capture 和标定程序 calibrate_camera。
  - jetson-toolchain.cmake：面向 Jetson ARM64 的交叉编译配置。
  - scripts/cross-deploy-run.sh：交叉构建、上传到 Jetson 并运行。

## 调参方式

算法阈值不通过命令行传入，直接改代码中的默认参数：

- 预处理参数：`ArmorPreprocessParams`
- 灯条筛选参数：`LightBarFilterParams`
- 装甲板配对参数：`ArmorMatcherParams`
- PnP 参数：`PnpSolverParams`
- 相机参数：`HikCameraConfig`

命令行只保留运行控制，例如相机编号、抓帧数量、保存路径和是否显示预览。

## PnP 标定参数

程序运行时只读 YAML，不再解析 `.npy`。默认标定文件路径是 `loadCalibration` 的默认实参：

```cpp
Calibration loadCalibration(const std::string& path = "config/camera_calibration.yml");
```

当前已将这两个文件转换到 `config/camera_calibration.yml`：

```text
C:/Users/MOS/Desktop/camera_matrix.npy
C:/Users/MOS/Desktop/dist_coeffs.npy
```

如果以后重新标定，直接用 `calibrate_camera` 工具生成这个 YAML（见下方「相机标定」一节）。如果只有 Python 算好的 `.npy`，也可以离线转换：

```bash
node tools/convert_calibration_npy.js camera_matrix.npy dist_coeffs.npy config/camera_calibration.yml
```

PnP 的 2D 点顺序是：

```text
left.top -> right.top -> right.bottom -> left.bottom
```

对应 3D 点以装甲板中心为原点，单位默认按毫米理解：

```text
(-w/2, -h/2, 0) -> (w/2, -h/2, 0) -> (w/2, h/2, 0) -> (-w/2, h/2, 0)
```

small/large 装甲板尺寸现在只是占位值，后面按实测尺寸改 `small_armor_width`、`small_armor_height`、`large_armor_width`、`large_armor_height` 即可。

## 编译

目标环境是 Linux/aarch64，并安装海康 MVS SDK、OpenCV 4 和 CMake。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

默认 SDK 路径为 `/opt/MVS`。如果 SDK 在其他位置：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMVCAM_ROOT=/path/to/MVS
cmake --build build -j
```

也可以分别指定 include 和 lib：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMVCAM_INCLUDE_DIR=/path/to/include \
  -DMVCAM_LIBRARY_DIR=/path/to/lib/aarch64
cmake --build build -j
```

如果 CMake 找不到 OpenCV，可以指定 OpenCV 的 CMake 配置目录：

```bash
cmake -S . -B build -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4
```

CMake 会把可执行文件输出到 `build/hik_capture`，并在构建后把 `config` 目录复制到 `build/config`。

## 运行

列出相机：

```bash
./build/hik_capture --list
```

抓 10 帧并保存到 `captures`：

```bash
./build/hik_capture --index 0 --frames 10 --output captures
```

连续运行并显示调试画面，不保存：

```bash
./build/hik_capture --index 0 --frames 0 --show --no-save
```

显示二值化调试画面：

```bash
./build/hik_capture --index 0 --frames 0 --show-binary --no-save
```

## 相机标定（棋盘格）

`calibrate_camera` 是独立的标定工具，和主程序一起由 CMake 构建，输出到 `build/calibrate_camera`。它固定使用 9×6 个内角点、25 mm 方格，并要求至少采集 20 个有效视角。标定结果写入 `config/camera_calibration.yml`，主程序无需改动即可读取。

实时标定（从相机取帧，需要有显示窗口）：

```bash
./build/calibrate_camera
```

预览窗口使用半分辨率检测棋盘格，采样时仍使用原始分辨率提取精确角点。按键：`SPACE` 采纳当前视角，`u` 撤销上一个，`ENTER` 标定，`ESC` 放弃退出。需要覆盖画面不同位置和角度采集至少 20 个视角；不足 20 个视角时不会写入标定文件。

离线标定（从已保存的图片目录，不需要相机）：

```bash
./build/calibrate_camera --images captures
```

终端会列出每张图是否检测到棋盘格，并打印整体 RMS 重投影误差、相机矩阵和畸变系数。目录中至少需要 20 张能检测到棋盘格的图片。运行目录要和主程序一致，以便相对路径 `config/camera_calibration.yml` 指向正确位置。

从开发机交叉编译并在 Jetson 上实时标定：

```bash
./scripts/cross-calibrate-run.sh
```

Jetson 只需连接相机和屏幕。预览画面显示在 Jetson 屏幕上，所有操作都在开发机终端完成：`SPACE` 采样、`u` 撤销、`ENTER` 标定、`q` 放弃。标定成功后，脚本会将生成的 YAML 自动下载到开发机的 `config/camera_calibration.yml`。

## 当前输出

每帧会在终端打印：

```text
frame=<id> hardwareTimestamp=<raw> size=<w>x<h> channels=<n> pixelType=<hex> rects=<n> lines=<n> lights=<n> armors=<n> poses=<n>
```

如果成功解出至少一个位姿，还会打印第一块装甲板的平移向量：

```text
first_tvec=(x,y,z)
```

调试画面中：

- 红色：`fitLine` 中心线
- 红 / 蓝 / 黄：最终筛选出的灯条框，黄色表示未知颜色，旁边的 `A=<value>` 是对应轮廓面积
- 白色：配对成功的装甲板候选
