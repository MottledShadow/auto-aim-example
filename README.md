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

- `include/hik_capture.hpp` / `src/hik_capture.cpp`：海康相机封装，只负责枚举、打开、配置、取帧和像素格式转换。
- `include/vision_pipeline.hpp`：视觉模块共享头——数据类型（`LightBar`、`Armor` 等）、各阶段参数与结果、`Calibration`，以及所有自由函数声明。
- `src/vision_pipeline.cpp`：编排层，`runPipeline` 串起四个阶段，外加跨阶段的角点工具 `armorCorners`。
- `src/armor_preprocessor.cpp`：图像预处理与轮廓几何提取（`preprocessFrame`）。
- `src/light_bar_filter.cpp`：灯条筛选与颜色识别（`filterLightBars`）。
- `src/armor_matcher.cpp`：灯条配对成装甲板候选（`matchArmors`）。
- `src/pnp_solver.cpp`：读取 YAML 标定（`loadCalibration`）与装甲板 PnP 位姿解算（`solvePnp`）。
- `include/debug_draw.hpp` / `src/debug_draw.cpp`：调试画面绘制。
- `config/camera_calibration.yml`：由 Python `.npy` 标定参数转换出的 OpenCV YAML。
- `tools/convert_calibration_npy.js`：离线转换 `.npy` 到 YAML 的小工具，不参与程序编译。
- `CMakeLists.txt`：CMake 构建入口。
- `src/main.cpp`：程序入口，含运行选项解析。
- `src/calibrate_main.cpp`：棋盘格相机标定工具 `calibrate_camera` 的入口，复用 `hik_capture` 取帧，输出 `config/camera_calibration.yml`。

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

预览窗口里的按键：`SPACE` 采纳当前视角，`u` 撤销上一个，`ENTER` 标定，`ESC` 放弃退出。需要覆盖画面不同位置和角度采集至少 20 个视角；不足 20 个视角时不会写入标定文件。

离线标定（从已保存的图片目录，不需要相机）：

```bash
./build/calibrate_camera --images captures
```

终端会列出每张图是否检测到棋盘格，并打印整体 RMS 重投影误差、相机矩阵和畸变系数。目录中至少需要 20 张能检测到棋盘格的图片。运行目录要和主程序一致，以便相对路径 `config/camera_calibration.yml` 指向正确位置。

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
