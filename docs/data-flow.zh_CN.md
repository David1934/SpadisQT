# SpadisQT — 数据流程

[English](data-flow.md) · [文档索引](README.zh_CN.md)

本文追踪一帧从传感器到屏幕/上位机的全过程，说明 depth16 格式与工作模式，并展示单帧控制流。

## 1. 帧处理流水线

![帧数据流](images/data_flow.svg)

1. **ADS6401 传感器**推流原始 MIPI 数据（`V4L2_PIX_FMT_SBGGR8`）。
2. **`V4L2`** 在其 `poll()` 循环中从 `/dev/video0` 出队 mmap 缓冲区，发出 `rx_new_frame(...)`。
3. **`FrameProcessThread`** 在 `new_frame_handle()` 中接收帧，调用
   `ADAPS_DTOF::dtof_frame_decode()`。
4. **`ADAPS_DTOF`** 调用闭源 `DepthMapWrapperProcessFrame()`，根据工作模式与构建选项产出：
   - **depth16** 缓冲区，
   - **灰度**缓冲区，
   - **点云**（`pc_pkt_t` 数组），和/或
   - **光斑直方图**（`SpotHistogram`）。
5. GUI 构建下，depth16 经 `ConvertDepthToColoredMap()` 着色为 `QImage`，作为
   `newFrameReady4Display()` 发往 **`MainWindow`**。
6. 若上位机请求了输出，解码后的缓冲区经
   **`Host_Communication::upload_frame_*()`** 通过 `libAdapsSender.so` 推送。

## 2. 单帧控制流

![单帧时序](images/capture_sequence.svg)

`FrameProcessThread::run()` 的工作循环反复执行：

1. `V4L2::Capture_frame()` → 对 video fd `poll()`，再 `VIDIOC_DQBUF` 取得原始缓冲区。
2. 发出 `rx_new_frame`，由 `new_frame_handle()`（同线程）处理。
3. **解码分支** —— 除非设置了 `skip_frame_decode`，否则调用 `dtof_frame_decode()`。
4. **显示分支** —— GUI 构建将 depth16 着色并发出 `newFrameReady4Display`。
5. **上传分支** —— 当上位机请求采集时，调用 `upload_frame_*()`。
6. `VIDIOC_QBUF` 重新入队缓冲区；循环继续。

许多步骤可在运行期经环境变量切换（见
[API 文档 §7](api-reference.zh_CN.md#7-运行期环境变量-commonh)）——例如 `skip_frame_decode`、
`save_frame_raw_data_enable`、`raw_file_replay_enable`、`force_framerate_fps`。

## 3. depth16 格式

输出是一种**改造过的 Android `DEPTH16`**（`adaps_dtof.h`）：

- 标准 Android DEPTH16 在高 3 位放置信度、低 13 位放置距离。
- SpadisQT 改为**低 14 位放距离**（`DEPTH_MASK = 0x3FFF`，`DEPTH_BITS = 14`）——把最大量程
  从 8.192 m 扩展到 **16.384 m**——**高 2 位放信度**（`CONFIDENCE_MASK = 0x3`）。

信度等级（`enum depth_confidence_level`）：`ANDROID_CONF_HIGH = 0`、`ANDROID_CONF_LOW = 1`、
`ANDROID_CONF_MEDIUM = 3`。

颜色映射默认使用 `RANGE_MIN = 30` mm .. `RANGE_MAX = 8192` mm；实时范围可经
`GlobalApplication` 的颜色映射 setter（以及上位机 `SET_COLORMAP_RANGE_PARAM` 命令）调整。
输出分辨率为 **210 × 160**（`OUTPUT_WIDTH/HEIGHT_4_DTOF_SENSOR`）。

## 4. 传感器类型与工作模式

`enum sensortype`（`common.h`）：`SENSOR_TYPE_RGB`、`SENSOR_TYPE_DTOF`。

`enum sensor_workmode`（`common.h`）：

| 工作模式 | 含义 |
|----------|------|
| `WK_DTOF_PHR` | dToF 局部/高速深度采集 |
| `WK_DTOF_PCM` | dToF 点云模式 |
| `WK_DTOF_FHR` | dToF 全直方图速率模式 |
| `WK_RGB_NV12` / `WK_RGB_YUYV` | RGB 传感器直通模式 |

`enum frame_data_type` 区分流经流水线的各类缓冲区（`FDATA_TYPE_DTOF_RAW_*`、
`FDATA_TYPE_DTOF_DECODED_DEPTH16`、`FDATA_TYPE_DTOF_DECODED_POINT_CLOUD`、
`FDATA_TYPE_DTOF_RAW_HISTOGRAM`，以及 RGB 类型等）。

## 5. 缓冲与时序常量（`v4l2.h` / `common.h`）

- `BUFFER_COUNT_4_DTOF_SENSOR = 12` 个 mmap 缓冲区（可经 `force_frame_buffer_count` 覆盖）。
- `MIPI_RAW_HEIGHT_4_DTOF_SENSOR = 32` 每次传输的原始 MIPI 行数。
- `FRAME_PROCESS_THREAD_INTERVAL_US = 10` µs 循环节拍。
- `DEFAULT_POLL_TIMEOUT_MS = -1`（永久等待；可经 `force_poll_timeout` 覆盖）。
- 每 `FRAME_COUNT_TO_READ_TEMPERATURE = 10` 帧读取一次温度；超出
  `CHIP_TEMPERATURE_MIN/MAX_THRESHOLD`（15 °C .. 90 °C）时告警。
