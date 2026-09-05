# hik_camera

- [HikCamera](#hikcamera)
  - [HikCamera()](#hikcamera-1)
  - [grabFrame](#grabframe)
  - [capture](#capture)
  - [cleanup](#cleanup)
- [HikCameraFrame](#hikcameraframe)

## HikCamera

海康相机驱动

采用内部采集线程轮询取图，保留最新帧供外部获取。

### HikCamera()

相机初始化

构造函数按照海康官方开发指南初始化相机：先初始化 SDK，再枚举相机、创建句柄、打开设备、设置参数并开始取流，随后启动采集线程。

### grabFrame

获取并处理图像

通过 SDK 内部缓存获取图像，将图像格式转换为 BGR8，将硬件时间戳转换为纳秒，并记录帧号，最后释放图像缓存。采集线程循环执行取图过程。

### capture

获取最新帧

外部通过 `capture()` 获取采集线程提供的最新帧，输出数据使用 `HikCameraFrame` 结构体。

### cleanup

释放相机资源

结束采集时关闭采集线程，停止取流、关闭设备、销毁句柄并释放 SDK 资源。

## HikCameraFrame

相机帧数据

包含：

- 当前帧图像 `image`
- 帧号 `frameNumber`
- 转换后的硬件时间戳 `timestampNs`，单位为纳秒
