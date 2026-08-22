
# detector 识别模块

RM 装甲板识别：一帧 `cv::Mat` 进，识别出的装甲板（含数字与相机系位姿）出。编成静态库 `detector`，下游链接它即可（`inc/` 经 CMake PUBLIC 暴露）。

```
detector/
  inc/     geometry_detector.hpp / number_classifier.hpp / pnp_solver.hpp / armor_detector.hpp
  src/     geometry_detector.cpp / number_classifier.cpp / pnp_solver.cpp / armor_detector.cpp
  model/   mlp.onnx + label.txt（数字分类模型与标签）
```

`inc/geometry_detector.hpp` 既放几何检测器 `GeometryDetector`，也放它产出的共享类型（`LightColor`/各 `*Params`/`ContourCandidate`/`PreprocessResult`/`LightBar`/`Armor`）；`number_classifier.hpp` / `pnp_solver.hpp` 各自 `#include "geometry_detector.hpp"` 取这些类型；要一把拿全量就 include `armor_detector.hpp`。

## 流水线

```text
cv::Mat
  → GeometryDetector::preprocess       灰度→二值→形态学→findContours→minAreaRect/fitLine，得候选
  → GeometryDetector::filterLightBars   按面积/长短比/角度/填充率筛灯条，并在轮廓内比红蓝定颜色
  → GeometryDetector::matchArmors       同色灯条两两配对成装甲，区分大/小装甲
  → NumberClassifier::classify  矫正 ROI → ONNX MLP → softmax，写回 number/confidence 并按规则去留
  → PnpSolver::solve            按装甲物理尺寸生成 3D 点 + 灯条 2D 点，solvePnP 得位姿
```

调试不再分阶段：读磁盘图片的 `detector_debug_file` 一次跑完整流水线、每张图输出一个含五阶段标注图的目录；接相机实时的 `detector_debug_camera` 单窗口按 1–5 键切换/叠加各阶段标注层。见根 README 的运行说明。

## 两个使用入口

模块对外就两个头：

- **`armor_detector.hpp` —— 生产入口（一把拿全量）**。给追踪器、主循环用。一个 facade 类 `ArmorDetector`：构造一次，每帧 `detect(frame)` 直接拿 `std::vector<ArmorPose>`，内部按顺序跑完 5 个阶段。`ArmorPose`（armor + rvec + tvec）就是 detector→tracker 的边界。它 include 了 `geometry_detector.hpp`+`number_classifier.hpp`+`pnp_solver.hpp`，include 它即拿到全部接口。

  ```cpp
  auto_aim::ArmorDetector detector(auto_aim::loadCalibration());
  // 每帧：
  std::vector<auto_aim::ArmorPose> poses = detector.detect(frame);
  ```

- **阶段 API**。给调试工具用：`geometry_detector.hpp`（`GeometryDetector` 及共享数据结构）、`number_classifier.hpp`（`NumberClassifier`）、`pnp_solver.hpp`（`PnpSolver`），按需 include，可逐阶段抽取产物做可视化和自检（两个 debug 工具就这么用）。

调参对两个入口都一样：改 `*Params` 的头文件默认值。`ArmorDetector` 只需传相机标定，其余走默认。

## 三个阶段类

- **`GeometryDetector`**（`geometry_detector.hpp`）：无状态，只做预处理/灯条/配对三个几何阶段；三个参数结构体是公开成员，三个阶段各一方法（`preprocess`/`filterLightBars`/`matchArmors`）。
- **`NumberClassifier`**（`number_classifier.hpp`）：构造时加载一次 ONNX 网络 + 标签表（有状态、开销大），之后每帧 `classify` 复用；`diagnose` 给单块装甲的完整推理产物供调试。
- **`PnpSolver`**（`pnp_solver.hpp`）：构造时持有标定与参数，每帧 `solve` 对配对装甲解位姿。标定用 `loadCalibration(path)` 从 YAML 读，`error` 非空表示失败。

## 调参

阈值**不走命令行**，直接改头文件里的 struct 默认值：

| 结构体 | 头文件 | 管什么 |
|---|---|---|
| `PreprocessParams` | geometry_detector.hpp | 二值化阈值 |
| `LightBarFilterParams` | geometry_detector.hpp | 灯条面积/长短比/角度/填充率范围，目标颜色 |
| `LightBarMatcherParams` | geometry_detector.hpp | 配对的长度比/角度差/中心 y 差/中心距比例、大装甲判定 |
| `NumberClassifierParams` | number_classifier.hpp | 矫正尺寸/ROI、模型与标签路径、置信度阈值 |
| `PnpSolverParams` | pnp_solver.hpp | 装甲物理尺寸(mm)、solvePnP 方法 |

`LightBarMatcherParams::maxLightAngleDiffDeg` 标了 `// TODO 上车重点调整`。

## PnP 点序契约

2D 图像点顺序（来自左右灯条的 top/bottom）：

```text
left.top → right.top → right.bottom → left.bottom
```

对应 3D 点以装甲中心为原点，单位 mm：

```text
(-w/2, -h/2, 0) → (w/2, -h/2, 0) → (w/2, h/2, 0) → (-w/2, h/2, 0)
```

`PnpSolverParams` 里 small/large 装甲的 `*Width`/`*Height` 目前是占位值，上车后按实测尺寸改。

## 模型与运行时路径

`NumberClassifierParams` 默认 `model/mlp.onnx`、`model/label.txt`，`PnpSolver` 构造时读写死的 `config/camera_calibration.yml`（标定路径不走命令行，`PnpSolver::error()` 报加载失败）——都相对**运行时工作目录**解析（Jetson 上是 `~/cross-snapshot`），与仓库里 `detector/model/` 的源码位置无关。部署脚本会把 `detector/model/` 送到远端 `model/`。
