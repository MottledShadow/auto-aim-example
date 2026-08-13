# auto_aim_example

RM 装甲板识别示例工程：海康工业相机取帧 → OpenCV 视觉流水线（预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP 位姿解算）。识别器已完成，下一步是追踪器。

## 目录结构

```
detector/     识别模块（静态库 detector）：inc/ src/ model/ —— 见 detector/README.md
hik_camera/   海康相机后台线程驱动 HikCamera（LatestImagesOnly）
debug_draw/   调试可视化（灯条/装甲指标叠加、fitToScreen）+ latest_slot.hpp（最新帧槽）
tools/        snapshot.cpp（拍照）、calibrate_camera.cpp（棋盘格标定）
tests/        hik_camera_test（相机冒烟）、detector_debug_file/camera（识别器调试）
config/       camera_calibration.yml（相机内参/畸变）
scripts/      交叉编译 + 部署到 Jetson 的脚本
```

算法步骤、调参含义、PnP 点序契约在 **[detector/README.md](detector/README.md)**。

## 命名规范

| 种类 | 规范 | 例 |
|---|---|---|
| 类/结构体/枚举类型 | `PascalCase` | `GeometryDetector`、`ArmorPose` |
| 枚举成员 | `PascalCase` | `LightColor::Red` |
| 方法 / 自由函数 | `camelCase` | `filterLightBars`、`loadCalibration` |
| 局部变量 / 参数 / 结构体字段 | `camelCase` | `aspectRatio`、`binaryThreshold` |
| 私有类成员 | `camelCase_`（尾下划线） | `pnpSolver_`、`captureThread_` |
| 公开类数据成员 | `camelCase`（无尾下划线） | `preprocessParams` |
| 编译期常量 | `kPascalCase` | `kEpsilon`、`kRequiredViews` |
| 命名空间 | `snake_case` | `auto_aim` |
| 头/源文件 | `snake_case`，名与主类对齐 | `geometry_detector.hpp`↔`GeometryDetector` |
| 目录 | `snake_case` | `debug_draw/` |
| CMake target | `snake_case`，与源文件名 stem 对齐 | `calibrate_camera`←`calibrate_camera.cpp` |
| Shell 脚本 | `kebab-case`（符合 shell 习惯的例外） | `cross-run.sh` |

## 构建 —— 交叉编译到 Jetson (aarch64)

本机没有原生 MVS SDK，原生配置会在 `find_library` 处 FATAL。实际构建路径是交叉编译：SDK/OpenCV/编译器都在 sysroot，由 `jetson-toolchain.cmake` 组装。固定用 `build/cross`（脚本和 `.vscode` 硬编码了它）。

```bash
cmake -S . -B build/cross -DCMAKE_TOOLCHAIN_FILE=jetson-toolchain.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/cross                          # detector 库 + 全部可执行文件
cmake --build build/cross --target detector_debug_file      # 单独构建
```

产物是 aarch64 二进制，**不能在开发机直接跑**，须部署到 Jetson。

## Target

| target | 说明 |
|---|---|
| `detector` | 识别静态库（`add_subdirectory(detector)`） |
| `snapshot` | 拍照工具：预览窗口，SPACE 存图到 `captures/`，q/ESC 退出 |
| `calibrate_camera` | 棋盘格标定，输出 `config/camera_calibration.yml` |
| `hik_camera_test` | 相机硬件冒烟测试（手写 `main`，需连真机） |
| `detector_debug_file` | 读磁盘图片的识别器调试：一次跑完整流水线，每图一目录输出五阶段标注图，只依赖 OpenCV |
| `detector_debug_camera` | 接相机实时的识别器调试：连相机实时取帧，每帧跑完整流水线，单窗口按 1-5 键切换/叠加各阶段标注层 |

调试不再分阶段：`detector_debug_file` 每张图输出 `1_preprocess..5_pnp.png` 一整套；`detector_debug_camera` 单窗口按 `1`=预处理 `2`=灯条 `3`=配对 `4`=数字 `5`=PnP 切换/叠加图层，`q/ESC` 退出。阈值不走命令行，改头文件默认值（见 detector/README.md）。

## 部署脚本（需 SSH 里配好名为 `jetson` 的主机别名）

- `scripts/cross-run.sh <camera|file>` —— 识别器调试的统一入口，交叉构建 `detector_debug_<mode>` → scp → 在 Jetson 运行。`file` 批处理 `captures/`；`camera` 开实时窗口（`DISPLAY=:0`）。因为整条流水线都跑，每次都把 `detector/model/`、`config/camera_calibration.yml` 送过去。
- `scripts/cross-snapshot-run.sh` —— 部署运行 `snapshot`，采集的照片留在 Jetson 的 `~/cross-snapshot/captures/`。

Jetson 上工作目录是 `~/cross-snapshot`，模型/标定按相对路径 `model/`、`config/` 解析。

## 相机标定

`calibrate_camera` 固定用 11×8 内角点、60 mm 方格，至少 20 个有效视角，结果写入 `config/camera_calibration.yml`。

实时标定（从相机取帧，需显示窗口）：

```bash
./calibrate_camera
```

按键：`SPACE` 采纳当前视角、`u` 撤销、`ENTER` 标定、`ESC` 放弃。

离线标定（从已保存图片目录）：

```bash
./calibrate_camera --images captures
```

目录里至少 20 张能检测到棋盘格的图片；运行目录需和主程序一致，以便相对路径 `config/camera_calibration.yml` 指向正确位置。

## 相机模块

`hik_camera/` 是唯一的相机封装：`HikCamera` 类，后台 `captureLoop` 线程 + `condition_variable`，`MV_GrabStrategy_LatestImagesOnly`，被 `snapshot`/`calibrate_camera`/`hik_camera_test`/`detector_debug_camera` 使用。改相机行为都在这里。
