# detector 识别模块

RM 装甲板识别：一帧 `cv::Mat` 进，识别出的装甲板（含数字与相机系位姿）出。编成静态库 `detector`，下游链接它即可（`inc/` 经 CMake PUBLIC 暴露）。

```
detector/
  inc/     detector.hpp / classifier.hpp / pnp.hpp
  src/     detector.cpp / classifier.cpp / pnp.cpp
  model/   mlp.onnx + label.txt（数字分类模型与标签）
```

`inc/detector.hpp` 是伞头：末尾 `#include "classifier.hpp"` 与 `"pnp.hpp"`，下游只需 `#include "detector.hpp"` 就能拿到全部接口；公共类型（`Armor` 等）在伞头前定义，化解环形包含。

## 流水线

```text
cv::Mat
  → Detector::preprocess       灰度→二值→形态学→findContours→minAreaRect/fitLine，得候选
  → Detector::filterLightBars   按面积/长短比/角度/填充率筛灯条，并在轮廓内比红蓝定颜色
  → Detector::matchArmors       同色灯条两两配对成装甲，区分大/小装甲
  → NumberClassifier::classify  矫正 ROI → ONNX MLP → softmax，写回 number/confidence 并按规则去留
  → PnpSolver::solve            按装甲物理尺寸生成 3D 点 + 灯条 2D 点，solvePnP 得位姿
```

分阶段调试用离线测试的 `<stage>`（`preprocess|lightbar|armor|number|pnp`）逐段跑，见根 README 的运行说明。

## 三个类

- **`Detector`**（`detector.hpp`）：无状态，三个参数结构体是公开成员，三个阶段各一方法（`preprocess`/`filterLightBars`/`matchArmors`）。
- **`NumberClassifier`**（`classifier.hpp`）：构造时加载一次 ONNX 网络 + 标签表（有状态、开销大），之后每帧 `classify` 复用；`diagnose` 给单块装甲的完整推理产物供调试。
- **`PnpSolver`**（`pnp.hpp`）：构造时持有标定与参数，每帧 `solve` 对配对装甲解位姿。标定用 `loadCalibration(path)` 从 YAML 读，`error` 非空表示失败。

## 调参

阈值**不走命令行**，直接改头文件里的 struct 默认值：

| 结构体 | 头文件 | 管什么 |
|---|---|---|
| `PreprocessParams` | detector.hpp | 二值化阈值 |
| `LightBarFilterParams` | detector.hpp | 灯条面积/长短比/角度/填充率范围，目标颜色 |
| `LightBarMatcherParams` | detector.hpp | 配对的长度比/角度差/中心 y 差/中心距比例、大装甲判定 |
| `NumberClassifierParams` | classifier.hpp | 矫正尺寸/ROI、模型与标签路径、置信度阈值 |
| `PnpSolverParams` | pnp.hpp | 装甲物理尺寸(mm)、solvePnP 方法 |

`LightBarMatcherParams::max_light_angle_diff_deg` 标了 `// TODO 上车重点调整`。

## PnP 点序契约

2D 图像点顺序（来自左右灯条的 top/bottom）：

```text
left.top → right.top → right.bottom → left.bottom
```

对应 3D 点以装甲中心为原点，单位 mm：

```text
(-w/2, -h/2, 0) → (w/2, -h/2, 0) → (w/2, h/2, 0) → (-w/2, h/2, 0)
```

`PnpSolverParams` 里 small/large 装甲的 `*_width`/`*_height` 目前是占位值，上车后按实测尺寸改。

## 模型与运行时路径

`NumberClassifierParams` 默认 `model/mlp.onnx`、`model/label.txt`，`loadCalibration` 默认 `config/camera_calibration.yml`——都相对**运行时工作目录**解析（Jetson 上是 `~/cross-snapshot`），与仓库里 `detector/model/` 的源码位置无关。部署脚本会把 `detector/model/` 送到远端 `model/`。
