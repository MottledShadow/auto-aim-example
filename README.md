# auto_aim_example

RM 装甲板识别示例工程：海康工业相机取帧 → OpenCV 视觉流水线（预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP 位姿解算）。识别器已完成，下一步是追踪器。

## 目录结构

```
detector/     识别模块（静态库 detector）：inc/ src/ model/ —— 见 detector/README.md
hik_camera/   海康相机后台线程驱动 HikCamera（LatestImagesOnly）
debug_draw/   调试可视化（灯条/装甲指标叠加、fitToScreen）+ latest_slot.hpp（最新帧槽）
tools/        snapshot（拍照）、calibrate_camera（棋盘格标定）的 main
tests/        hik_camera_test（相机冒烟）、pipeline_offline/online_test（分阶段调试）
config/       camera_calibration.yml（相机内参/畸变）
scripts/      交叉编译 + 部署到 Jetson 的脚本
```

算法步骤、调参含义、PnP 点序契约在 **[detector/README.md](detector/README.md)**。

## 构建 —— 交叉编译到 Jetson (aarch64)

本机没有原生 MVS SDK，原生配置会在 `find_library` 处 FATAL。实际构建路径是交叉编译：SDK/OpenCV/编译器都在 sysroot，由 `jetson-toolchain.cmake` 组装。固定用 `build/cross`（脚本和 `.vscode` 硬编码了它）。

```bash
cmake -S . -B build/cross -DCMAKE_TOOLCHAIN_FILE=jetson-toolchain.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/cross                          # detector 库 + 全部可执行文件
cmake --build build/cross --target pipeline_offline_test   # 单独构建
```

产物是 aarch64 二进制，**不能在开发机直接跑**，须部署到 Jetson。

## Target

| target | 说明 |
|---|---|
| `detector` | 识别静态库（`add_subdirectory(detector)`） |
| `snapshot` | 拍照工具：预览窗口，SPACE 存图到 `captures/`，q/ESC 退出 |
| `calibrate_camera` | 棋盘格标定，输出 `config/camera_calibration.yml` |
| `hik_camera_test` | 相机硬件冒烟测试（手写 `main`，需连真机） |
| `pipeline_offline_test` | 离线分阶段调试：读磁盘图片，命令行 `<stage>` 选跑到哪一阶段，只依赖 OpenCV |
| `pipeline_online_test` | 在线分阶段调试：连相机实时取帧，单窗口显示 |

`<stage>` 取 `preprocess|lightbar|armor|number|pnp`。阈值不走命令行，改头文件默认值（见 detector/README.md）。

## 部署脚本（需 SSH 里配好名为 `jetson` 的主机别名）

- `scripts/cross-run.sh <stage> <offline|online>` —— 分阶段调试的统一入口，交叉构建 `pipeline_<mode>_test` → scp → 在 Jetson 运行。`offline` 批处理 `captures/`；`online` 开实时窗口（`DISPLAY=:0`）。`number`/`pnp` 阶段会一并把 `detector/model/`、`config/camera_calibration.yml` 送过去。
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

`hik_camera/` 是唯一的相机封装：`HikCamera` 类，后台 `captureLoop` 线程 + `condition_variable`，`MV_GrabStrategy_LatestImagesOnly`，被 `snapshot`/`calibrate_camera`/`hik_camera_test`/`pipeline_online_test` 使用。改相机行为都在这里。
