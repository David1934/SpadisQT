# SpadisQT — 软件架构

[English](architecture.md) · [文档索引](README.zh_CN.md)

SpadisQT 是一个**用户态 Qt 5 应用**，它将 ADAPS ADS6401 dToF 传感器的原始 MIPI 帧解码为
深度 / 灰度 / 点云数据，进行可视化，并可选地推流到上位机。它是 `ads6401` 内核驱动与两个
预编译闭源 aarch64 库之上的一层薄封装。

![架构总览](images/arch_overview.svg)

## 1. 分层

自底向上：

1. **硬件** —— ADS6401 dToF 传感器模组（四种类型之一：*Spot*、*Flood*、*Big-FoV Flood*、
   *Big-FoV Flood V2*），通过 MIPI-CSI 及 I²C/控制总线连接。
2. **Linux 内核** —— V4L2/media 子系统提供 `/dev/video0` + `/dev/media0` 用于取流；
   `ads6401` 驱动提供 `/dev/ads6401` 用于控制 ioctl（EEPROM、曝光、寄存器、模组静态数据、
   配置脚本与 ROI-SRAM 下发）。
3. **预编译库**（`/vendor/lib64`）—— `libadaps_swift_decode.so`（解码算法）与
   `libAdapsSender.so`（上位机传输）。两者均为 aarch64 闭源二进制；应用通过
   `-Wl,-rpath,/vendor/lib64/` 链接。
4. **应用进程** —— `SpadisQT`（GUI）或 `SpadisQT_console`（无界面），由同一份源码构建。
5. **上位机（可选）** —— ADAPS 上位机工具，经 `libAdapsSender.so` 通信。

## 2. 核心类

| 类 | 文件 | 职责 |
|----|------|------|
| `V4L2` | `v4l2.cpp/.h` | 管理 V4L2 设备：按关键字 `ads6401` 发现子设备、`VIDIOC_*` 配置、缓冲区 mmap/入队、`poll()` 采集循环、帧出队。发出 `rx_new_frame` / `update_info`。 |
| `FrameProcessThread` | `FrameProcessThread.cpp/.h` | 工作线程 `QThread`。构造并持有 `V4L2` + `ADAPS_DTOF`，连接它们的信号，在 `run()` 中跑采集循环，逐帧解码，发出用于显示和/或上传的帧。 |
| `ADAPS_DTOF` | `adaps_dtof.cpp/.h` | 经 `depthmapwrapper.h` 封装解码库。把原始数据转为 depth16 / 灰度 / 点云，应用颜色映射（`ConvertDepthToColoredMap`），产出直方图。 |
| `Misc_Device` | `misc_device.cpp/.h` | 对 `/dev/ads6401` 的全部 ioctl：EEPROM/标定读取、曝光参数、寄存器读写、模组静态数据、配置脚本 + ROI-SRAM 下发。 |
| `Host_Communication` | `host_comm.cpp/.h` | 经 `libAdapsSender.so` 与上位机通信的**单例**桥。接收命令并上传 raw/depth16/点云/直方图帧。 |
| `GlobalApplication` | `globalapplication.cpp/.h` | `QApplication`/`QCoreApplication` 子类，经重载的 `qApp` 宏访问。**全局状态持有者/注册表**：已选传感器类型与工作模式、颜色映射范围、已加载的标定缓冲、以及 `V4L2`/`Misc_Device` 单例指针。 |
| `MainWindow` | `mainwindow.cpp/.h/.ui` | 解码帧与状态的 GUI 接收方。在 console 构建下退化为普通 `QObject`，GUI 槽被编译排除。 |

`main.cpp` 通过 `QLockFile` 保证单实例启动，安装 Unix 信号处理（含 `backtrace()` 崩溃栈
转储），运行 `requirement_check()`（算法 `.so` 与 XML 配置必须存在），然后启动应用。

## 3. 线程模型

![线程模型](images/threading_model.svg)

共有两个线程：

- **主线程** —— 运行 Qt 事件循环；承载 `GlobalApplication`、`MainWindow`（仅 GUI）与
  `Host_Communication` 单例。
- **`FrameProcessThread`** —— 采集+解码工作线程。持有 `V4L2` 与 `ADAPS_DTOF`，二者归属于
  该工作线程。

跨线程的信号：

- `V4L2::rx_new_frame` → `FrameProcessThread::new_frame_handle` 运行在**工作线程内**
  （繁重的解码路径）。
- `FrameProcessThread::newFrameReady4Display` / `update_runtime_display` 以**队列连接**
  投递到**主线程**的 `MainWindow`，从而保证 GUI 更新线程安全。
- `Host_Communication::start_capture` / `stop_capture` 来自上位机传输回调，投递到工作线程。
- 帧上传（`upload_frame_*`）由工作线程**直接调用** `Host_Communication` 单例。

单帧时序见 [数据流程](data-flow.zh_CN.md)。

## 4. 编译期特性开关

整块子系统由两个 `.pro` 文件中的 `DEFINES` 控制。**当你修改某个用这些宏 `#if` 分支的
信号/槽时，务必同时修改所有分支。**

| 宏 | 作用 |
|----|------|
| `RUN_ON_EMBEDDED_LINUX` | 总开关 —— 编入 `adaps_dtof.cpp`、`host_comm.cpp`、`misc_device.cpp` 并链接两个算法 `.so`。不定义时仅构建 V4L2/RGB 骨架。 |
| `RUN_ON_RK3568` 与否 | 选择设备节点：`/dev/media0` + `/dev/video0`（RK3568）对 `/dev/media2` + `/dev/video22`（RK3588），见 `common.h`。 |
| `CONSOLE_APP_WITHOUT_GUI` | 无界面构建 —— `MainWindow` 变为 `QObject`；所有 GUI/`QImage` 路径被编译排除。仅由 `SpadisQT_console.pro` 设置。 |
| `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` | 完全去掉 `Host_Communication`。注意 `rx_new_frame` / `new_frame_handle` 依据此宏有**两种签名**。 |
| `ENABLE_POINTCLOUD_OUTPUT` | 启用点云输出（同时受算法库版本 ≥ 3.5.6 约束）。 |
| `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB` | 兼容旧版 `libadaps_swift_decode_3.3.2.so` 的兼容垫片。 |

两个 `.pro` 构建**同一份源码**，仅在 `gui`/`widgets` 对 `core`-only 的 Qt 模块、以及
`CONSOLE_APP_WITHOUT_GUI` 宏上不同——新增源文件时两者需保持同步。

## 5. 版本契约

两组版本号必须保持一致（见 [API 文档 §6](api-reference.zh_CN.md#6-解码库契约-depthmapwrapperh)）：

- **应用版本** —— `common.h` 中的 `VERSION_MAJOR/MINOR/REVISION` + `LAST_MODIFIED_TIME`
  （git 提交以 `v3.6.9_LM20260605A` 形式打标签）。
- **算法库版本** —— `depthmapwrapper.h` 中的 `ALGO_LIB_VERSION_MAJOR/MINOR/REVISION`。
  特性代码以 `ALGO_LIB_VERSION_CODE` 在 3.5.6 / 3.6.2 / 3.6.5 阈值上做 `#if` 守卫。
  **更换 `libadaps_swift_decode.so` 时务必同步调整这些宏**，否则版本门控的字段会与二进制
  不匹配。
