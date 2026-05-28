# auto_aim_example

RM 装甲板识别示例工程。当前项目从海康工业相机取帧，将图像转换为 OpenCV `cv::Mat`，再完成图像预处理、灯条筛选、灯条颜色识别、装甲板配对和 PnP 位姿解算。

## Pipeline

当前视觉流程在 `VisionPipeline` 中串联：

```text
HikCapture
  -> ArmorPreprocessor
  -> LightBarFilter
  -> ArmorMatcher
  -> PnpSolver
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
- `include/vision_pipeline.hpp` / `src/vision_pipeline.cpp`：视觉 pipeline 总入口。
- `include/armor_preprocessor.hpp` / `src/armor_preprocessor.cpp`：图像预处理和轮廓几何提取。
- `include/light_bar_filter.hpp` / `src/light_bar_filter.cpp`：灯条筛选和颜色识别。
- `include/armor_matcher.hpp` / `src/armor_matcher.cpp`：灯条配对成装甲板候选，并粗分 small/large。
- `include/pnp_solver.hpp` / `src/pnp_solver.cpp`：装甲板 PnP 位姿解算，读取 OpenCV YAML 标定文件。
- `include/armor_types.hpp`：`LightBar`、`Armor` 等基础数据结构。
- `include/app_options.hpp` / `src/app_options.cpp`：运行选项解析。
- `include/debug_draw.hpp` / `src/debug_draw.cpp`：调试画面绘制。
- `config/camera_calibration.yml`：由 Python `.npy` 标定参数转换出的 OpenCV YAML。
- `tools/convert_calibration_npy.js`：离线转换 `.npy` 到 YAML 的小工具，不参与程序编译。
- `CMakeLists.txt`：CMake 构建入口。
- `src/main.cpp`：程序入口。

## 调参方式

算法阈值不通过命令行传入，直接改代码中的默认参数：

- 预处理参数：`ArmorPreprocessParams`
- 灯条筛选参数：`LightBarFilterParams`
- 装甲板配对参数：`ArmorMatcherParams`
- PnP 参数：`PnpSolverParams`
- 相机参数：`HikCameraConfig`

命令行只保留运行控制，例如相机编号、抓帧数量、保存路径和是否显示预览。

## PnP 标定参数

程序运行时只读 YAML，不再解析 `.npy`。默认标定文件路径在 `PnpSolverParams::calibration_file`：

```cpp
std::string calibration_file = "config/camera_calibration.yml";
```

当前已将这两个文件转换到 `config/camera_calibration.yml`：

```text
C:/Users/MOS/Desktop/camera_matrix.npy
C:/Users/MOS/Desktop/dist_coeffs.npy
```

如果以后重新标定，可以重新生成 YAML：

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

## 当前输出

每帧会在终端打印：

```text
frame=<id> size=<w>x<h> channels=<n> pixelType=<hex> contours=<n> rects=<n> lines=<n> lights=<n> armors=<n> poses=<n>
```

如果成功解出至少一个位姿，还会打印第一块装甲板的平移向量：

```text
first_tvec=(x,y,z)
```

调试画面中：

- 绿色：候选最小外接矩形
- 红色：`fitLine` 中心线
- 红 / 蓝 / 黄：识别出的灯条，黄色表示未知颜色
- 白色：配对成功的装甲板候选
