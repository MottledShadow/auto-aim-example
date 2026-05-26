# auto_aim_example

这个仓库先放一个更干净的海康工业相机 OpenCV 取流示例，后面 RM 装甲板识别 pipeline 可以直接从 `cv::Mat` 接入。

## 现在有什么

- `src/hik_capture.cpp`：枚举海康 GigE/USB3 相机，打开指定相机，取流并转换成 OpenCV `cv::Mat`。
- `src/main.cpp`：命令行程序入口，把相机取帧、视觉 pipeline、保存和预览串起来。
- `include/app_options.hpp` / `src/app_options.cpp`：命令行参数解析。
- `include/debug_draw.hpp` / `src/debug_draw.cpp`：调试显示绘制。
- `include/vision_pipeline.hpp` / `src/vision_pipeline.cpp`：组合预处理、灯条筛选和装甲板配对。
- `include/armor_types.hpp`：装甲板识别中的基础数据结构，包含灯条 `LightBar` 和装甲板 `Armor`。
- `include/armor_preprocessor.hpp` / `src/armor_preprocessor.cpp`：每帧图像的灰度化、亮区二值化、开闭运算、轮廓提取、最小外接矩形和中心线提取。
- `include/light_bar_filter.hpp` / `src/light_bar_filter.cpp`：按面积、长短边比、fitLine 角度和外层轮廓填充率筛选灯条候选，并在轮廓 mask 内识别红/蓝颜色。
- `include/armor_matcher.hpp` / `src/armor_matcher.cpp`：按长度比、角度差、中心 y 差、中心距比例、无遮挡和同色规则配对装甲板。
- `Makefile`：不再写死官方 Sample 的相对路径，默认按 `/opt/MVS` 或 `MVCAM_COMMON_RUNENV` 找海康 MVS SDK。
- 输出图像默认保存为 `captures/frame_000000.png` 这种 PNG，方便调试预处理和灯条识别。

和官方 `RawDataFormatConvert_OpenCV4` 示例相比，这个版本去掉了交互式 `scanf` 和过时的 `IplImage`，支持连续取流、预览、手动曝光/增益、Bayer/RGB/BGR/Mono 到 OpenCV Mat 的转换，并用 RAII 自动释放相机句柄和帧缓存。

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
./build/hik_capture --index 0 --frames 0 --show --no-save --exposure-us 3000 --gain 8
```

调试预处理阈值和形态学参数：

```bash
./build/hik_capture --index 0 --frames 0 --show-binary --no-save --binary-threshold 180 --open-kernel 3 --close-kernel 3
```

调试灯条筛选阈值：

```bash
./build/hik_capture --index 0 --frames 0 --show --no-save --min-light-area 5 --max-light-area 1000000 --min-light-aspect 1.2 --max-light-aspect 50 --min-light-angle 0 --max-light-angle 45 --min-light-fill 0.25 --max-light-fill 1
```

调试灯条配对阈值：

```bash
./build/hik_capture --index 0 --frames 0 --show --no-save --max-armor-length-ratio 2 --max-armor-angle-diff 10 --max-armor-y-diff 40 --min-armor-distance-ratio 0.5 --max-armor-distance-ratio 8
```

后面接装甲板识别时，可以把 `convertFrameToBgrOrGray()` 返回的 `cv::Mat image` 作为预处理入口：颜色阈值、灯条轮廓、灯条配对、角点排序、PnP 都从这里往后接。
