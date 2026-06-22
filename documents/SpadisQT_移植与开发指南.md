# SpadisQT 移植与开发指南

> 本文面向**首次接触 SpadisQT 的工程师**，从零讲解如何把这套 ADS6401 dToF 参考应用编译、部署并跑起来，以及如何移植到自己的开发板。
> 配套阅读：[README（中文）](../README_zh_CN.md) · [SDK 用户手册 PDF](Ads6401_dToF_SDK_For_Linux_User_Guide.pdf) · [English version](SpadisQT_Porting_Guide_EN.md)

---

## 目录

1. [总体认识：这套软件是什么、依赖什么](#1-总体认识)
2. [开工前的硬件 / 软件清单](#2-开工前清单)
3. [第一步：确认开发板上的传感器已就绪](#3-第一步确认传感器已就绪)
4. [第二步：搭建编译环境](#4-第二步搭建编译环境)
5. [第三步：编译 SpadisQT](#5-第三步编译-spadisqt)
6. [第四步：配置 `adapsdepthsettings.xml`](#6-第四步配置-adapsdepthsettingsxml)
7. [第五步：部署到开发板](#7-第五步部署到开发板)
8. [第六步：运行与验证](#8-第六步运行与验证)
9. [移植到新开发板 / 新 SoC](#9-移植到新开发板--新-soc)
10. [上位机通信（可选）](#10-上位机通信可选)
11. [环境变量速查表](#11-环境变量速查表)
12. [编译期宏速查表](#12-编译期宏速查表)
13. [常见问题排查](#13-常见问题排查)

---

## 1. 总体认识

SpadisQT 是一层很薄的**用户态程序**，它本身不包含任何深度算法 —— 真正的解码工作由两个**预编译的闭源库**完成。理解这条依赖链是移植成功的前提：

```
ADS6401 传感器  ──MIPI──►  ads6401 内核驱动  ──►  /dev/video0、/dev/media0（取流）
                                              └──►  /dev/ads6401（控制 ioctl）
                                                         ▲
                                                         │
   SpadisQT（用户态） ─── V4L2 取流 ─── 解码 ─── 显示/上传
                              │           │
                              │           └── libadaps_swift_decode.so（深度算法，闭源）
                              └── libAdapsSender.so（上位机传输，闭源）
```

完整架构与数据流见 README：
- [架构总览](../vx_images/arch_overview.png)
- [帧数据流](../vx_images/data_flow.png)

**三件事必须同时具备，应用才能跑起来：**
1. 内核里有 `ads6401` 驱动，且 `/dev/video0`、`/dev/media0`、`/dev/ads6401` 节点存在；
2. `libadaps_swift_decode.so`、`libAdapsSender.so` 部署到 `/vendor/lib64/`；
3. `adapsdepthsettings.xml` 部署到 `/vendor/etc/camera/`，且其中的 `BufferWidth*` 与你的 SoC 匹配。

> 缺任意一项，程序会在启动时的 `requirement_check()` 直接报错退出（见 `main.cpp`）。

---

## 2. 开工前清单

### 硬件
- [ ] 一块嵌入式开发板（参考已验证平台：**RK3568 / Linux 5.10**，代码也内置 RK3588 路径）；
- [ ] 一颗通过 **MIPI-CSI** 接到该板的 **ADS6401 dToF 模块**（散点 / 小面阵 / 大 FoV / 大 FoV V2 任一）；
- [ ] 一根串口线或 SSH/ADB 通道，用于登录开发板（默认登录 `root` / `rockchip`）；
- [ ] （GUI 版需要）一块显示屏 / HDMI / LVDS，且系统具备显示后端。

### 软件
- [ ] 开发板内核**已集成 `ads6401` 驱动**（本仓库不含驱动，需从对应的内核 SDK 获取）；
- [ ] 一台 x86_64 Linux 编译主机；
- [ ] 目标架构的 **Qt 5.x**（交叉编译 SDK，推荐）；
- [ ] 本仓库自带文件：源代码、`libadaps_swift_decode.so`、`libAdapsSender.so`、`adapsdepthsettings.xml`。

> 💡 本仓库自带的 `.so` 是 **aarch64（ARM64）** 二进制，只能在 ARM 开发板上加载，**无法在 x86 主机上运行**。因此程序无法在 PC 上"本地跑"，必须部署到开发板。

---

## 3. 第一步：确认传感器已就绪

在编译任何东西之前，**先登录开发板**确认底层驱动 OK，否则即使编译成功也取不到流。

```bash
# 1) 控制节点是否存在
ls -l /dev/ads6401

# 2) V4L2 视频 / media 节点是否存在
ls -l /dev/video0 /dev/media0

# 3) 列出 V4L2 设备，确认能看到 ads6401 子设备
v4l2-ctl --list-devices

# 4) 查看 media 拓扑，应能在 entity 名称里看到包含 "ads6401" 的子设备
media-ctl -p -d /dev/media0 | grep -i ads6401
```

- `KEYWORD_4_DTOF_SENSOR_SUBDEV_NAME` 在 `common.h` 中定义为 `"ads6401"` —— `V4L2` 类正是靠这个关键字在 media 拓扑里找到传感器子设备。
- RK3568 默认节点：`/dev/media0` + `/dev/video0`；RK3588 默认：`/dev/media2` + `/dev/video22`（见 `common.h`，由编译期是否定义 `RUN_ON_RK3568` 决定）。

✅ 三类节点都在、`media-ctl` 能看到 `ads6401` 子设备，才进行下一步。

---

## 4. 第二步：搭建编译环境

SpadisQT 用 **qmake + make** 构建，需要**目标架构（aarch64）的 Qt 5.x**。有两条路线：

### 路线 A（推荐）：在 PC 上交叉编译
你需要一套面向开发板的 Qt5 交叉 SDK，通常由板厂提供，里面包含针对 aarch64 的 `qmake`、Qt 头文件与库（sysroot）。

```bash
# 让交叉版 qmake 进入 PATH（路径以你的 SDK 为准）
export PATH=/opt/your-board-qt5/bin:$PATH
qmake -query QT_VERSION      # 确认是 5.x
qmake -query QMAKE_XSPEC     # 确认是 aarch64/arm 交叉 spec
```

### 路线 B：直接在开发板上编译
若开发板自带 `gcc` + Qt5 开发包（`qmake`、`qtbase-dev` 等），可以把源码拷到板上直接 `qmake && make`。简单但编译慢，且很多精简 rootfs 不带编译器。

> ⚠️ **不要用 PC 自带的 x86 版 Qt（如 `apt install qtbase5-dev`）来编 SpadisQT**——那样编出来的是 x86 程序，无法在 ARM 板上运行，且链接不了仓库里的 aarch64 `.so`。

---

## 5. 第三步：编译 SpadisQT

仓库里有两个工程文件，分别产出两个目标：

| 工程文件 | 产物 | 区别 |
|----------|------|------|
| `SpadisQT.pro` | `SpadisQT` | GUI 版，链接 `gui` + `widgets` |
| `SpadisQT_console.pro` | `SpadisQT_console` | 无界面版，定义 `CONSOLE_APP_WITHOUT_GUI`，仅链接 `core` |

> 两个 `.pro` 共用同一套源文件，**只靠 `CONSOLE_APP_WITHOUT_GUI` 宏区分**。如果你新增源文件，记得两个 `.pro` 都要加。

### 编译命令

```bash
cd /path/to/SpadisQT

# —— GUI 版 ——
qmake SpadisQT.pro
make -j"$(nproc)"

# —— 无界面版（建议放到独立目录，避免 .o 冲突）——
mkdir -p build_console && cd build_console
qmake ../SpadisQT_console.pro
make -j"$(nproc)"
```

### Debug / Release
由 qmake 的 `CONFIG` 决定（见 `.pro` 文件）：
- **Release**（默认/`CONFIG+=release`）：`-O3 -flto -fno-exceptions`，定义 `BUILD_4_RELEASE`；
- **Debug**（`CONFIG+=debug`）：保留 `-rdynamic`，使 `main.cpp` 里的崩溃回溯（`backtrace()`）能解析出函数名，便于定位问题。

```bash
qmake CONFIG+=debug SpadisQT.pro && make -j"$(nproc)"   # 调试期建议
```

### 目标平台（RK3568 vs RK3588）
默认按 RK3568 编译（`.pro` 中 `DEFINES += RUN_ON_RK3568`）。移植到 RK3588 时需改用 RK3588 的设备节点，详见[第 9 节](#9-移植到新开发板--新-soc)。

编译成功后，你应得到可执行文件 `SpadisQT` 和 `SpadisQT_console`。

---

## 6. 第四步：配置 `adapsdepthsettings.xml`

这是**最容易在移植时踩坑的一步**。该文件是解码算法库的配置，其中两个缓冲宽度**与 SoC 强相关**，配错会导致解码异常或花屏。

```xml
<!-- SM8450/windows:1032  SM8250:1040  SM8550:1040  海思:1056  RK3568/RK3588:1280 -->
<BufferWidthPHR>1280</BufferWidthPHR>

<!-- SM8450/windows:4104  SM8250:4112  SM8550:4112  海思:4128  RK3568/RK3588:4352 -->
<BufferWidthFHR>4352</BufferWidthFHR>
```

| SoC 平台 | `BufferWidthPHR` | `BufferWidthFHR` |
|----------|------------------|------------------|
| **RK3568 / RK3588** | **1280** | **4352** |
| 高通 SM8450 / Windows | 1032 | 4104 |
| 高通 SM8250 | 1040 | 4112 |
| 高通 SM8550 | 1040 | 4112 |
| 海思 Hisilicon | 1056 | 4128 |

其他常用项：
- `ConvolutionType`：`0`=CPU、`1`=NEON、`2`=DSP/AVX。ARM 平台一般用 **`1`（NEON）**；
- `IsUndistort`：是否做去畸变，默认 `true`；
- `TimeBinInterval`：时间 bin 间隔，可选 `125 / 250 / 500`；
- `FrameAccumulateCount`：多帧累加数，默认 `1`。

> 用 RK3568 的板子务必确认是 `1280 / 4352`。仓库里自带的 XML 已是 RK 值，可直接用。

### 表外新 SoC 如何实测缓冲宽度

如果你的 SoC 不在上表里，就到硬件上实测。`BufferWidthFHR` 必须等于 V4L2 驱动**实际给出的每行 MIPI 总字节数**（有效载荷 + padding），而不是有效载荷宽度。`V4L2::Capture_frame()`（`v4l2.cpp`）在第一帧时按 `total_bytes_per_line = bytesused / raw_height` 算出该值并打日志。运行一次应用，读 `NOTICE` 行，例如在 RK3568 上：

```
------frame raw size: 4104 X 32, bits_per_pixel: 8, payload_bytes_per_line: 4104, total_bytes_per_line: 4352, padding_bytes_per_line: 248, frame_buffer_size: 139264---
```

这里 `total_bytes_per_line = 4352`，即把 `BufferWidthFHR` 设为 `4352`（248 个 padding 字节让每行比 4104 字节的有效载荷更长；算法库需要这个带 padding 的宽度才能正确跳到下一行）。`BufferWidthPHR` 用 PHR 模式抓一帧、同法得出。

---

## 7. 第五步：部署到开发板

把 5 个文件放到固定路径（程序会到这些固定路径找它们）：

```bash
# 1) 在开发板上建目录
mkdir -p /vendor/lib64 /vendor/etc/camera /data/vendor/camera

# 2A) SSH 方式（在 PC 上执行，替换为你的产物路径与板子 IP）
scp libadaps_swift_decode.so libAdapsSender.so  root@<板子IP>:/vendor/lib64/
scp adapsdepthsettings.xml                      root@<板子IP>:/vendor/etc/camera/
scp SpadisQT SpadisQT_console                   root@<板子IP>:/usr/bin/

# 2B) 或 ADB 方式
adb push libadaps_swift_decode.so /vendor/lib64/
adb push libAdapsSender.so        /vendor/lib64/
adb push adapsdepthsettings.xml   /vendor/etc/camera/
adb push SpadisQT                 /usr/bin/
adb push SpadisQT_console         /usr/bin/

# 3) 赋可执行权限（在开发板上执行）
chmod +x /usr/bin/SpadisQT /usr/bin/SpadisQT_console
```

部署后开发板上的文件布局应如下（参见 [部署示意图](../vx_images/deployment.png)）：

| 路径 | 内容 |
|------|------|
| `/usr/bin/SpadisQT`、`SpadisQT_console` | 两个可执行文件 |
| `/vendor/lib64/libadaps_swift_decode.so`、`libAdapsSender.so` | 算法 + 传输库（程序以 `-rpath /vendor/lib64` 链接，会自动到此目录找库） |
| `/vendor/etc/camera/adapsdepthsettings.xml` | 算法配置 |
| `/data/vendor/camera/` | 运行期 dump 目录 |

> `/vendor` 分区如果是只读，需要先 `mount -o remount,rw /vendor`，或把库放到 rootfs 可写位置并相应调整。

---

## 8. 第六步：运行与验证

```bash
# 无界面版（最容易先跑通，推荐先用它验证链路）
SpadisQT_console

# GUI 版（需要显示屏 + 显示后端）
SpadisQT
```

### 启动时会发生什么
1. `QLockFile` 保证**单实例**运行（重复启动第二个会直接退出）；
2. `requirement_check()` 检查 `/vendor/lib64/libadaps_swift_decode.so` 与 `/vendor/etc/camera/adapsdepthsettings.xml` 是否存在 —— 缺哪个就报哪个；
3. 打开 V4L2 设备、开流，工作线程 `FrameProcessThread` 开始取流并解码。

### GUI 版没有显示？
GUI 程序需要一个 Qt 显示后端。如果板子没有 X11/Wayland，可尝试 EGLFS 或 LinuxFB：

```bash
QT_QPA_PLATFORM=eglfs SpadisQT
# 或
QT_QPA_PLATFORM=linuxfb SpadisQT
```

### 开调试日志
程序默认只打印关键日志。需要详细日志时打开 `debug_info_enable`：

```bash
debug_info_enable=true SpadisQT_console
```

### 快速验证取流是否正常
```bash
# 落几帧原始数据 + 深度数据到 /data/vendor/camera 看看链路是否通
save_frame_raw_data_enable=true save_frame_depth16_enable=true SpadisQT_console
ls -l /data/vendor/camera/
```

✅ 看到帧率日志、`/data/vendor/camera/` 里有数据落盘，即说明采集+解码链路打通。

---

## 9. 移植到新开发板 / 新 SoC

按以下顺序逐项落实：

1. **内核驱动**：先在新板内核里集成并跑通 `ads6401` 驱动（设备树里配好 MIPI、I2C、复位/电源 GPIO），确认 `/dev/video*`、`/dev/media*`、`/dev/ads6401` 出现。这一步不在本仓库范围内，需找对应内核 SDK。
2. **设备节点路径**：如果节点不是默认的 `/dev/video0` + `/dev/media0`，修改 `common.h`：
   - RK3568 分支：`MEDIA_DEVNAME_4_DTOF_SENSOR` / `VIDEO_DEV_4_DTOF_SENSOR`；
   - RK3588 走 `#else` 分支（`/dev/media2` + `/dev/video22`），并启用 `VIDIOC_S_FMT_INCLUDE_VIDIOC_SUBDEV_S_FMT`（RK3588 上 `VIDIOC_S_FMT` 会一并设置 sensor 格式）；
   - 切换平台靠是否定义 `RUN_ON_RK3568`（在 `.pro` 中）。
3. **算法配置**：按[第 6 节](#6-第四步配置-adapsdepthsettingsxml)的表，把 `BufferWidthPHR/FHR` 改成新 SoC 的值，`ConvolutionType` 选对（ARM 用 NEON=1）。
4. **预编译库**：确认 `libadaps_swift_decode.so`、`libAdapsSender.so` 与新平台 ABI 兼容（都是 aarch64）。若算法库版本变化，需同步 `depthmapwrapper.h` 中的 `ALGO_LIB_VERSION_*` 宏 —— 否则版本门控的代码会编译不进来。
5. **Qt 后端**：确认新板的 Qt 显示后端（GUI 版）。
6. **重新编译 + 部署 + 验证**：回到第 5～8 节。

> 老算法库兼容：若必须搭配旧版 `libadaps_swift_decode.so`，在 `.pro` 里打开 `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB`。仓库还保留了 `libadaps_swift_decode_3.3.2.so` 供回退参考。

---

## 10. 上位机通信（可选）

如果需要把帧数据传给 PC 端工具（或由 PC 端下发命令、下载标定数据），就会用到 `Host_Communication` + `libAdapsSender.so`。

- 这是一个**单例**，通过回调接收上位机命令：启动/停止采集、读写寄存器、更新 EEPROM、下载 walk-error / spot-offset / 参考距离 / 镜头内参 / ROI-SRAM 标定数据、设置配色区间、重启、对时等；
- 反向把 raw / depth16 / 点云 / 直方图帧上传给上位机。

如果你的产品**完全不需要上位机**，可在 `.pro` 中定义 `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION`，整块通信代码（含 `host_comm.cpp`）将不参与编译。

> ⚠️ 注意：`STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` 会改变 `rx_new_frame` / `new_frame_handle` 的**函数签名**（带不带 `frame_buffer_param_t` 参数）。改这两个信号/槽时，两个分支都要同步修改。

---

## 11. 环境变量速查表

所有变量都在 `common.h` 中以 `ENV_VAR_*` 定义，运行时设为 `true`（或具体数值）即可，无需重新编译。下面按用途分类列出常用项。

**数据落盘 / 回放**
| 变量 | 作用 |
|------|------|
| `save_frame_raw_data_enable` | 落原始帧数据 |
| `save_frame_depth16_enable` | 落 depth16 数据 |
| `save_frame_pointcloud_enable` | 落点云数据 |
| `save_frame_histogram_data_enable` | 落直方图数据 |
| `save_depth_txt_enable` | 以文本形式落深度 |
| `save_eeprom_enable` | 落 EEPROM 数据 |
| `raw_file_replay_enable` | 用本地 raw 文件回放（脱离传感器调试） |
| `depth16_file_replay_enable` | 用本地 depth16 文件回放 |
| `enable_algo_lib_dump_data` | 让算法库落中间数据到 `/tmp/OfflineData/` |

**图像处理开关**
| 变量 | 作用 |
|------|------|
| `mirror_x_enable` / `mirror_y_enable` | 水平 / 垂直镜像 |
| `enable_expand_pixel` | 像素扩展（算法库内处理） |
| `disable_compose_subframe` | 关闭子帧合成 |
| `disable_walk_error` | 关闭 walk-error 补偿 |

**强制参数**（覆盖默认值，调试用）
| 变量 | 作用 |
|------|------|
| `force_framerate_fps` | 强制帧率 |
| `force_poll_timeout` | 强制 poll 超时（ms） |
| `force_coarseExposure` / `force_fineExposure` / `force_grayExposure` | 强制各类曝光值 |
| `force_laserExposurePeriod` | 强制激光曝光周期 |
| `force_row_search_range` / `force_column_search_range` | 强制 SPAD 搜索范围 |
| `force_frame_buffer_count` | 强制帧缓冲数 |
| `force_get_histogram_count` | 强制获取直方图数量 |

**日志 / 跟踪**
| 变量 | 作用 |
|------|------|
| `debug_info_enable` | 打开 `DBG_INFO` 详细日志 |
| `trace_algo_lib_decode_costtime` | 跟踪算法解码耗时 |
| `trace_output_frame_rate` | 跟踪输出帧率 |
| `trace_roi_sram_switch` | 跟踪 ROI-SRAM 切换 |
| `frame_drop_check_enable` | 丢帧检测 |
| `skip_frame_decode` / `skip_frame_process` | 跳过解码 / 处理（性能基线测试） |

> 还有一批 `dump_*`（如 `dump_module_static_data`、`dump_eeprom_data`、`dump_capture_req_param`、`dump_depth_map_frame_interval` 等）用于一次性转储各类调试信息，完整列表见 `common.h`。

---

## 12. 编译期宏速查表

在 `.pro` 文件里通过 `DEFINES +=` 设置，控制整块功能是否编入。

| 宏 | 作用 |
|----|------|
| `RUN_ON_EMBEDDED_LINUX` | **总开关**。开启后才编入 `adaps_dtof.cpp`、`host_comm.cpp`、`misc_device.cpp` 并链接两个 `.so`。不开则只有 V4L2/RGB 骨架 |
| `RUN_ON_RK3568` | 选用 RK3568 的设备节点；不定义则走 RK3588 分支 |
| `CONFIG_VIDEO_ADS6401` | 启用 ADS6401 相关代码 |
| `ENABLE_POINTCLOUD_OUTPUT` | 启用点云输出（同时受算法库版本 ≥ 3.5.6 门控） |
| `CONSOLE_APP_WITHOUT_GUI` | 无界面构建，`MainWindow` 退化为 `QObject`，GUI/QImage 代码不编入（仅 `SpadisQT_console.pro` 设置） |
| `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` | 去掉上位机通信整块（注意会改 `rx_new_frame` 签名） |
| `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB` | 兼容旧版算法库 |

> 另：`common.h` 中的 `VERSION_*` 是 App 版本（如 `3.6.7_LM20260402A`），`depthmapwrapper.h` 中的 `ALGO_LIB_VERSION_*` 是算法库版本。**更换 `.so` 时务必同步后者**，否则按版本号 `#if` 门控的功能代码会编译异常。

---

## 13. 常见问题排查

| 现象 | 可能原因与排查 |
|------|----------------|
| 启动即报 `Adaps algo lib <...> does not exist` | `libadaps_swift_decode.so` 没放到 `/vendor/lib64/`，或路径不对 |
| 启动即报 `config file <...> does not exist` | `adapsdepthsettings.xml` 没放到 `/vendor/etc/camera/` |
| 运行时报找不到 `.so`（`cannot open shared object`） | 库未部署，或 `/vendor/lib64` 只读未挂可写；确认 `-rpath` 指向 `/vendor/lib64` |
| 程序起来但**取不到帧 / poll 超时** | 先回到[第 3 节](#3-第一步确认传感器已就绪)：`/dev/video0`、`/dev/ads6401` 是否存在、`media-ctl` 能否看到 `ads6401` 子设备；传感器是否真正接好、上电 |
| 找不到传感器子设备 | `common.h` 的 `KEYWORD_4_DTOF_SENSOR_SUBDEV_NAME`（`ads6401`）与 media 拓扑里的名字不一致，或设备节点路径配错 |
| 画面**花屏 / 深度明显错误** | `adapsdepthsettings.xml` 的 `BufferWidthPHR/FHR` 与 SoC 不匹配（RK 应为 `1280/4352`）；或 `ConvolutionType` 选错 |
| GUI 版起不来 / 黑屏 | 缺显示后端，尝试 `QT_QPA_PLATFORM=eglfs` 或 `linuxfb`；确认板子有屏 |
| 帧率偏低 | 用 `trace_algo_lib_decode_costtime`、`trace_output_frame_rate` 定位瓶颈；确认 `ConvolutionType=1`（NEON） |
| 第二次启动直接退出 | 这是单实例保护（`QLockFile`），正常现象；先结束已有进程 |
| 程序崩溃想看调用栈 | 用 **Debug 版**编译（保留 `-rdynamic`），崩溃时 `main.cpp` 的信号处理会打印带函数名的回溯 |
| 怀疑是传感器还是算法的问题 | 用 `raw_file_replay_enable` 回放已知正常的 raw 文件，把传感器因素排除掉 |

---

## 还有问题？

- 翻阅 [Ads6401 dToF SDK for Linux 用户手册（PDF）](Ads6401_dToF_SDK_For_Linux_User_Guide.pdf)；
- 联系 [深圳灵明光子（ADAPS Photonics）](https://adapsphotonics.com/)。

> 关于"开发板是否支持 Qt"这类基础环境问题，请自行研究 —— 这部分不在本项目支持范围内。
