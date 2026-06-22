<div align="center">

# SpadisQT — Reference App for the ADAPS ADS6401 dToF Sensor

**A Qt 5 demo application that captures, decodes and visualizes depth data from the ADAPS ADS6401 direct-Time-of-Flight (dToF) sensor on embedded Linux.**

[简体中文](README_zh_CN.md) · [Porting & Development Guide](documents/SpadisQT_Porting_Guide_EN.md) · [移植与开发指南](documents/SpadisQT_移植与开发指南.md)

[![License](https://img.shields.io/badge/License-LGPLv3-blue.svg)](https://opensource.org/licenses/LGPL-3.0)
[![Platform](https://img.shields.io/badge/Platform-Embedded%20Linux-green.svg)](#requirements)
[![Qt](https://img.shields.io/badge/Qt-5.x-brightgreen.svg)](https://doc.qt.io/qt-5/)
[![SoC](https://img.shields.io/badge/Tested-RK3568%20%C2%B7%20Linux%205.10-orange.svg)](#requirements)

</div>

---

## 1. What is SpadisQT?

`SpadisQT` is the official reference application for the **ADS6401 dToF sensor** from [ADAPS Photonics (灵明光子)](https://adapsphotonics.com/). It runs on **embedded Linux** and demonstrates the full sensor pipeline:

1. Capture raw MIPI data from the Swift dToF sensor through the **V4L2** framework.
2. Decode the raw data into **depth / grayscale / point-cloud** using the proprietary `libadaps_swift_decode.so` algorithm library.
3. Render depth as an **RGB color map** for intuitive visualization, and optionally **stream frames to a host PC tool**.

It ships in two flavors built from the same code base:

| Target | Qt modules | UI | Use case |
|--------|-----------|----|----------|
| **`SpadisQT`** | `core` + `gui` + `widgets` | Full GUI window | Boards with a display / framebuffer |
| **`SpadisQT_console`** | `core` only | Headless | Pipes / host-driven / no display |

> ⚠️ Because it relies on Linux **V4L2** APIs, SpadisQT **cannot run on Windows**. It has been tested on **RK3568 with the Linux 5.10 kernel** (RK3588 device-node paths are also wired in).

### Supported dToF modules

The ADS6401 chip drives four module types. The `ads6401` kernel driver reports the active type via the `ADAPS_GET_DTOF_MODULE_STATIC_DATA` ioctl:

- **Spot** module
- **Small-Flood** module
- **Big FoV Flood** module
- **Big FoV V2 Flood** module

### Live screenshots

| GUI — depth color map | GUI — runtime view |
|---|---|
| ![SpadisQT](vx_images/172133286252452.png) | ![SpadisQT](vx_images/47092456821829.png) |

---

## 2. Architecture

SpadisQT is a thin userspace layer over the `ads6401` kernel driver and two prebuilt aarch64 libraries.

![Architecture overview](vx_images/arch_overview.png)

| Component | File | Responsibility |
|-----------|------|----------------|
| `V4L2` | `v4l2.cpp/.h` | Sensor discovery, V4L2 setup, mmap buffer queue, `poll()` capture loop. Emits `rx_new_frame`. |
| `FrameProcessThread` | `FrameProcessThread.cpp/.h` | Worker `QThread`. Owns `V4L2` + `ADAPS_DTOF`, drives decode, emits frames for display / upload. |
| `ADAPS_DTOF` | `adaps_dtof.cpp/.h` | Wraps `libadaps_swift_decode.so`: raw → depth16 / grayscale / point cloud, plus the depth→RGB color map. |
| `Misc_Device` | `misc_device.cpp/.h` | All `ioctl` traffic to `/dev/ads6401` (EEPROM, exposure, registers, module static data, config script, ROI-SRAM). |
| `Host_Communication` | `host_comm.cpp/.h` | **Singleton** bridge to the host PC over `libAdapsSender.so` (commands in, frames out). |
| `GlobalApplication` | `globalapplication.cpp/.h` | Global state / registry, reachable via the overridden `qApp` macro. |
| `MainWindow` | `mainwindow.cpp/.h/.ui` | GUI receiver of decoded frames; degrades to a plain `QObject` in console builds. |

### Frame data flow

![Data flow](vx_images/data_flow.png)

> **Depth16 format** — a modified Android `DEPTH16`: the low **14 bits** carry the distance (extending the max range to **16.384 m**) and the high **2 bits** carry the confidence level (`DEPTH_MASK` / `CONFIDENCE_MASK` in `adaps_dtof.h`).

> 🧭 **Going deeper?** The [developer documentation](docs/README.md) adds a threading model, a single-frame sequence diagram, and a full API reference. All diagrams are generated from Graphviz sources via [`tools/gen_diagrams.sh`](tools/gen_diagrams.sh).

---

## 3. Requirements

**Hardware**
- An embedded board with an **ADS6401 dToF module** connected over MIPI-CSI (reference: RK3568, RK3588).
- The board's Linux kernel must include the **`ads6401` driver**, exposing `/dev/video0` + `/dev/media0` (V4L2) and `/dev/ads6401` (control ioctls).

**Software**
- **Qt 5.x** for the target architecture — either a cross-compilation SDK (recommended) or a native Qt toolchain on the board. Both shipping binaries are **aarch64**, and the two prebuilt libraries are aarch64-only, so a real target build needs the **board's aarch64 Qt sysroot** (apt's host Qt5 cannot link them).
- The two prebuilt aarch64 libraries shipped in this repo: `libadaps_swift_decode.so`, `libAdapsSender.so`.
- OpenSSL + zlib (`-lssl -lcrypto -lz`) for the target.
- For the GUI build: a working display backend on the board (X11, Wayland, EGLFS or LinuxFB).

> ℹ️ Make sure your board actually supports Qt before starting — that setup is outside the scope of this project.
> The full dependency list and a host-side **compile-check** (validate the sources compile on an x86_64 box without a sysroot) are in [docs/build-environment.md](docs/build-environment.md).

---

## 4. Quick start

```bash
# 1. Build (with an aarch64 Qt5 cross-toolchain in PATH)
qmake SpadisQT.pro            # GUI target
make -j"$(nproc)"
qmake SpadisQT_console.pro    # headless target
make -j"$(nproc)"

# 2. Create target directories (on the board)
mkdir -p /vendor/lib64 /vendor/etc/camera /data/vendor/camera

# 3. Deploy (SSH example — or use `adb push`)
scp libadaps_swift_decode.so libAdapsSender.so  root@<board_ip>:/vendor/lib64/
scp adapsdepthsettings.xml                      root@<board_ip>:/vendor/etc/camera/
scp SpadisQT SpadisQT_console                   root@<board_ip>:/usr/bin/

# 4. Run (on the board)
chmod +x /usr/bin/SpadisQT /usr/bin/SpadisQT_console
SpadisQT                     # GUI    (needs a display)
SpadisQT_console             # headless
```

📖 **New to this?** The step-by-step **[Porting & Development Guide](documents/SpadisQT_Porting_Guide_EN.md)** walks through hardware verification, toolchain setup, configuration, deployment, tuning and troubleshooting from scratch.

> 🧪 **No sysroot yet?** You can still validate that the sources compile on a plain x86_64 Linux host (`sudo apt-get install -y qtbase5-dev qtbase5-dev-tools libssl-dev zlib1g-dev`, then `qmake` + `make`). The final link will fail on the aarch64 `.so` — that's expected. Details in [docs/build-environment.md](docs/build-environment.md).

---

## 5. Deployment layout

![Deployment](vx_images/deployment.png)

| Path on board | Content |
|---------------|---------|
| `/usr/bin/SpadisQT`, `/usr/bin/SpadisQT_console` | The two executables |
| `/vendor/lib64/libadaps_swift_decode.so`, `libAdapsSender.so` | Algorithm + host-transport libraries (the binary links with `-rpath /vendor/lib64`) |
| `/vendor/etc/camera/adapsdepthsettings.xml` | Algorithm config — **must match your SoC** (see below) |
| `/data/vendor/camera/` | Runtime dump directory (used by the `dump`/`save` env-var toggles) |

> 🔧 **Per-SoC config knob.** In `adapsdepthsettings.xml`, `BufferWidthPHR` / `BufferWidthFHR` are platform-specific. For **RK3568 / RK3588** use `1280` / `4352` respectively. The file lists the correct values for SM8450/SM8250/SM8550/Hisilicon as well — set these before porting to a new SoC.

---

## 6. Configuration & tuning

- **Compile-time switches** (set in the `.pro` files) gate whole subsystems: `RUN_ON_EMBEDDED_LINUX`, `RUN_ON_RK3568` vs RK3588 device nodes, `CONSOLE_APP_WITHOUT_GUI`, `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION`, `ENABLE_POINTCLOUD_OUTPUT`, `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB`.
- **Runtime environment variables** (defined in `common.h`, prefixed `ENV_VAR_*`) toggle data dumping, file replay, mirroring, forced parameters and verbose logging — e.g. `debug_info_enable=true`, `save_frame_raw_data_enable=true`, `mirror_x_enable=true`, `raw_file_replay_enable=true`, `force_framerate_fps=30`.

See the porting guide for the full reference tables.

---

## 7. Documentation

**Developer reference** (the code internals) — [`docs/`](docs/README.md):

- **[Architecture](docs/architecture.md)** — layered design, core classes, threading, compile-time switches.
- **[Data Flow](docs/data-flow.md)** — the frame pipeline, depth16 format, work modes.
- **[API Reference](docs/api-reference.md)** — class APIs, the decode-lib & host-protocol contracts, env-var toggles.
- **[Build Environment](docs/build-environment.md)** — dependencies, cross-build, host compile-check.

**Integration guides** (getting it onto a board):

- **[Porting & Development Guide (EN)](documents/SpadisQT_Porting_Guide_EN.md)** — full hands-on guide for first-time integrators.
- **[移植与开发指南 (中文)](documents/SpadisQT_移植与开发指南.md)** — 中文完整移植开发文档。
- **[Ads6401 dToF SDK for Linux — User Guide (PDF)](documents/Ads6401_dToF_SDK_For_Linux_User_Guide.pdf)** — the SDK reference manual.

> Every diagram in this repo is generated from Graphviz sources in [`tools/diagrams/`](tools/diagrams/) by [`tools/gen_diagrams.sh`](tools/gen_diagrams.sh) — edit the `.dot`, not the image.

---

## 8. Support & License

For technical questions, contact **[ADAPS Photonics](https://adapsphotonics.com/)**.

SpadisQT is licensed under **[GNU LGPLv3](https://opensource.org/licenses/LGPL-3.0)** and uses Qt under the **[Qt LGPL](https://doc.qt.io/archives/qt-5.15/lgpl.html)**.
