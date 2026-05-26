# auto_aim_example

这个仓库先放一个更干净的海康工业相机 OpenCV 取流示例，后面 RM 装甲板识别 pipeline 可以直接从 `cv::Mat` 接入。

## 现在有什么

- `src/hik_capture.cpp`：枚举海康 GigE/USB3 相机，打开指定相机，取流并转换成 OpenCV `cv::Mat`。
- `include/armor_types.hpp`：装甲板识别中的基础数据结构，包含灯条 `LightBar` 和装甲板 `Armor`。
- `include/armor_preprocessor.hpp` / `src/armor_preprocessor.cpp`：每帧图像的灰度化、亮区二值化、开闭运算、轮廓提取、最小外接矩形和中心线提取。
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

后面接装甲板识别时，可以把 `convertFrameToBgrOrGray()` 返回的 `cv::Mat image` 作为预处理入口：颜色阈值、灯条轮廓、灯条配对、角点排序、PnP 都从这里往后接。
