# auto_aim_example

这个仓库实现 RM 装甲板识别的基础 pipeline，并提供一个海康工业相机取流入口。相机帧会被转换成 OpenCV `cv::Mat`，再进入预处理、灯条筛选、颜色识别和装甲板配对。

## 现在有什么

- `src/hik_capture.cpp`：只负责海康 GigE/USB3 相机枚举、打开、取帧和像素格式转换。
- `src/main.cpp`：程序入口，串起相机取帧、视觉 pipeline、保存和预览。
- `include/app_options.hpp` / `src/app_options.cpp`：运行选项解析。
- `include/debug_draw.hpp` / `src/debug_draw.cpp`：调试显示绘制。
- `include/vision_pipeline.hpp` / `src/vision_pipeline.cpp`：组合预处理、灯条筛选和装甲板配对。
- `include/armor_types.hpp`：基础数据结构，包含 `LightBar` 和 `Armor`。
- `include/armor_preprocessor.hpp` / `src/armor_preprocessor.cpp`：灰度化、亮区二值化、开闭运算、轮廓提取、最小外接矩形和中心线提取。
- `include/light_bar_filter.hpp` / `src/light_bar_filter.cpp`：按面积、长短边比、fitLine 角度、填充率筛选灯条，并在轮廓 mask 内识别红/蓝颜色。
- `include/armor_matcher.hpp` / `src/armor_matcher.cpp`：按长度比、角度差、中心 y 差、中心距比例、无遮挡和同色规则配对装甲板。

## 调参

调试阈值不走命令行，直接改代码里的默认值：

- 预处理阈值：`ArmorPreprocessParams`
- 灯条筛选阈值：`LightBarFilterParams`
- 装甲板配对阈值：`ArmorMatcherParams`
- 相机曝光、增益、宽高：`HikCameraConfig`

## 编译

目标环境是安装了海康 MVS SDK 和 OpenCV 4 的 Linux/aarch64 机器。

```bash
make print-config
make
```

如果 SDK 没装在 `/opt/MVS`，可以手动传路径：

```bash
make MVCAM_ROOT=/path/to/MVS
```

或者分别指定：

```bash
make MVCAM_INCLUDE=/path/to/include MVCAM_LIB_DIR=/path/to/lib/aarch64
```

## 运行

列出相机：

```bash
./build/hik_capture --list
```

抓 10 帧并保存：

```bash
./build/hik_capture --index 0 --frames 10 --output captures
```

连续预览，不保存：

```bash
./build/hik_capture --index 0 --frames 0 --show --no-save
```

查看二值化调试画面：

```bash
./build/hik_capture --index 0 --frames 0 --show-binary --no-save
```
