# SpadisQT Porting & Development Guide

> This guide is written for **engineers meeting SpadisQT for the first time**. It walks through building, deploying and running the ADS6401 dToF reference app from scratch, and porting it to your own board.
> Companion reading: [README (EN)](../README.md) · [SDK User Guide PDF](Ads6401_dToF_SDK_For_Linux_User_Guide.pdf) · [中文版](SpadisQT_移植与开发指南.md)

---

## Contents

1. [Big picture: what it is and what it depends on](#1-big-picture)
2. [Pre-flight hardware / software checklist](#2-pre-flight-checklist)
3. [Step 1 — Verify the sensor is ready on the board](#3-step-1--verify-the-sensor)
4. [Step 2 — Set up the build environment](#4-step-2--set-up-the-build-environment)
5. [Step 3 — Build SpadisQT](#5-step-3--build-spadisqt)
6. [Step 4 — Configure `adapsdepthsettings.xml`](#6-step-4--configure-adapsdepthsettingsxml)
7. [Step 5 — Deploy to the board](#7-step-5--deploy-to-the-board)
8. [Step 6 — Run and verify](#8-step-6--run-and-verify)
9. [Porting to a new board / SoC](#9-porting-to-a-new-board--soc)
10. [Host communication (optional)](#10-host-communication-optional)
11. [Environment-variable reference](#11-environment-variable-reference)
12. [Compile-time define reference](#12-compile-time-define-reference)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. Big picture

SpadisQT is a thin **userspace layer**. It contains no depth algorithm of its own — the real decode work is done by two **prebuilt closed-source libraries**. Understanding this dependency chain is the key to a successful port:

```
ADS6401 sensor  ──MIPI──►  ads6401 kernel driver  ──►  /dev/video0, /dev/media0  (capture)
                                                  └──►  /dev/ads6401            (control ioctls)
                                                             ▲
                                                             │
   SpadisQT (userspace) ─── V4L2 capture ─── decode ─── display / upload
                                │              │
                                │              └── libadaps_swift_decode.so (depth algo, closed)
                                └── libAdapsSender.so (host transport, closed)
```

Full architecture & data flow are in the README:
- [Architecture overview](../vx_images/arch_overview.png)
- [Frame data flow](../vx_images/data_flow.png)

**Three things must be in place for the app to run:**
1. The `ads6401` driver is in the kernel, and `/dev/video0`, `/dev/media0`, `/dev/ads6401` all exist;
2. `libadaps_swift_decode.so` and `libAdapsSender.so` are deployed to `/vendor/lib64/`;
3. `adapsdepthsettings.xml` is deployed to `/vendor/etc/camera/`, with `BufferWidth*` matching your SoC.

> If any of these is missing, the app fails fast in `requirement_check()` at startup (see `main.cpp`).

---

## 2. Pre-flight checklist

### Hardware
- [ ] An embedded board (validated reference: **RK3568 / Linux 5.10**; RK3588 paths also wired in);
- [ ] An **ADS6401 dToF module** wired to it over **MIPI-CSI** (Spot / Small-Flood / Big-FoV / Big-FoV V2);
- [ ] A console / SSH / ADB channel to log in (default login `root` / `rockchip`);
- [ ] (GUI build only) a display + a working display backend.

### Software
- [ ] The board kernel **already integrates the `ads6401` driver** (not shipped in this repo — get it from the matching kernel SDK);
- [ ] An x86_64 Linux build host;
- [ ] **Qt 5.x** for the target architecture (cross-compilation SDK, recommended);
- [ ] The files shipped in this repo: the sources, `libadaps_swift_decode.so`, `libAdapsSender.so`, `adapsdepthsettings.xml`.

> 💡 The bundled `.so` files are **aarch64 (ARM64)** binaries — they only load on an ARM board and **cannot run on an x86 host**. The app therefore cannot run "locally" on a PC; it must be deployed to the board.

---

## 3. Step 1 — Verify the sensor

Before building anything, **log into the board** and confirm the low-level driver works — otherwise even a perfect build won't capture frames.

```bash
# 1) Control node present?
ls -l /dev/ads6401

# 2) V4L2 video / media nodes present?
ls -l /dev/video0 /dev/media0

# 3) List V4L2 devices; you should see the ads6401 subdevice
v4l2-ctl --list-devices

# 4) Inspect the media topology; an entity name should contain "ads6401"
media-ctl -p -d /dev/media0 | grep -i ads6401
```

- `KEYWORD_4_DTOF_SENSOR_SUBDEV_NAME` in `common.h` is `"ads6401"` — the `V4L2` class uses this keyword to locate the sensor subdevice in the media graph.
- RK3568 default nodes: `/dev/media0` + `/dev/video0`; RK3588 default: `/dev/media2` + `/dev/video22` (see `common.h`, selected by whether `RUN_ON_RK3568` is defined).

✅ Only proceed once all nodes exist and `media-ctl` shows the `ads6401` subdevice.

---

## 4. Step 2 — Set up the build environment

SpadisQT builds with **qmake + make** and needs **Qt 5.x for the target (aarch64)**. Two routes:

### Route A (recommended): cross-compile on a PC
You need a Qt5 cross SDK for your board (usually provided by the board vendor) containing an aarch64 `qmake`, Qt headers and libraries (sysroot).

```bash
# Put the cross qmake on PATH (adjust to your SDK)
export PATH=/opt/your-board-qt5/bin:$PATH
qmake -query QT_VERSION      # confirm 5.x
qmake -query QMAKE_XSPEC     # confirm an aarch64/arm cross spec
```

### Route B: build natively on the board
If the board ships `gcc` + Qt5 dev packages (`qmake`, `qtbase-dev`, …), copy the sources over and `qmake && make` directly. Simple but slow, and many trimmed rootfs images have no compiler.

> ⚠️ **Do not** build SpadisQT with your PC's native x86 Qt (e.g. `apt install qtbase5-dev`) — that produces an x86 binary that won't run on the ARM board and can't link the aarch64 `.so` files in this repo.

---

## 5. Step 3 — Build SpadisQT

Two project files produce two targets:

| Project file | Output | Difference |
|--------------|--------|------------|
| `SpadisQT.pro` | `SpadisQT` | GUI build, links `gui` + `widgets` |
| `SpadisQT_console.pro` | `SpadisQT_console` | Headless, defines `CONSOLE_APP_WITHOUT_GUI`, links `core` only |

> Both `.pro` files share the same sources and differ **only by the `CONSOLE_APP_WITHOUT_GUI` define**. If you add a source file, add it to both.

### Build commands

```bash
cd /path/to/SpadisQT

# —— GUI build ——
qmake SpadisQT.pro
make -j"$(nproc)"

# —— Headless build (use a separate dir to avoid .o clashes) ——
mkdir -p build_console && cd build_console
qmake ../SpadisQT_console.pro
make -j"$(nproc)"
```

### Debug / Release
Selected by qmake `CONFIG` (see the `.pro` files):
- **Release** (default / `CONFIG+=release`): `-O3 -flto -fno-exceptions`, defines `BUILD_4_RELEASE`;
- **Debug** (`CONFIG+=debug`): keeps `-rdynamic` so the crash backtrace (`backtrace()`) in `main.cpp` can resolve function names.

```bash
qmake CONFIG+=debug SpadisQT.pro && make -j"$(nproc)"   # recommended while debugging
```

### Target platform (RK3568 vs RK3588)
Defaults to RK3568 (`DEFINES += RUN_ON_RK3568` in the `.pro`). For RK3588 you must switch to its device nodes — see [Section 9](#9-porting-to-a-new-board--soc).

On success you get the executables `SpadisQT` and `SpadisQT_console`.

---

## 6. Step 4 — Configure `adapsdepthsettings.xml`

This is the **easiest step to get wrong when porting**. The file configures the decode library, and two buffer widths are **SoC-specific**; a mismatch causes corrupted or wrong depth output.

```xml
<!-- SM8450/windows:1032  SM8250:1040  SM8550:1040  Hisilicon:1056  RK3568/RK3588:1280 -->
<BufferWidthPHR>1280</BufferWidthPHR>

<!-- SM8450/windows:4104  SM8250:4112  SM8550:4112  Hisilicon:4128  RK3568/RK3588:4352 -->
<BufferWidthFHR>4352</BufferWidthFHR>
```

| SoC | `BufferWidthPHR` | `BufferWidthFHR` |
|-----|------------------|------------------|
| **RK3568 / RK3588** | **1280** | **4352** |
| Qualcomm SM8450 / Windows | 1032 | 4104 |
| Qualcomm SM8250 | 1040 | 4112 |
| Qualcomm SM8550 | 1040 | 4112 |
| Hisilicon | 1056 | 4128 |

Other common keys:
- `ConvolutionType`: `0`=CPU, `1`=NEON, `2`=DSP/AVX. On ARM use **`1` (NEON)**;
- `IsUndistort`: enable undistortion, default `true`;
- `TimeBinInterval`: `125 / 250 / 500`;
- `FrameAccumulateCount`: multi-frame accumulation, default `1`.

> For RK3568 boards make sure it's `1280 / 4352`. The XML shipped in this repo already uses the RK values.

### Deriving the widths for an unlisted SoC

If your SoC isn't in the table, measure it on hardware. `BufferWidthFHR` must equal the
**total bytes per MIPI line** (payload + padding) that the V4L2 driver actually delivers —
not the payload width. `V4L2::Capture_frame()` (`v4l2.cpp`) computes it on the first frame
as `total_bytes_per_line = bytesused / raw_height` and logs it. Run the app once and read the
`NOTICE` line, e.g. on RK3568:

```
------frame raw size: 4104 X 32, bits_per_pixel: 8, payload_bytes_per_line: 4104, total_bytes_per_line: 4352, padding_bytes_per_line: 248, frame_buffer_size: 139264---
```

Here `total_bytes_per_line = 4352`, so set `BufferWidthFHR = 4352` (the 248 padding bytes
make the line longer than the 4104-byte payload; the algo library needs the padded width to
step to the next row correctly). Derive `BufferWidthPHR` the same way from a PHR-mode capture.

---

## 7. Step 5 — Deploy to the board

Place 5 files at fixed paths (the app looks for them there):

```bash
# 1) Create dirs on the board
mkdir -p /vendor/lib64 /vendor/etc/camera /data/vendor/camera

# 2A) Via SSH (run on the PC; replace with your output path and board IP)
scp libadaps_swift_decode.so libAdapsSender.so  root@<board_ip>:/vendor/lib64/
scp adapsdepthsettings.xml                      root@<board_ip>:/vendor/etc/camera/
scp SpadisQT SpadisQT_console                   root@<board_ip>:/usr/bin/

# 2B) Or via ADB
adb push libadaps_swift_decode.so /vendor/lib64/
adb push libAdapsSender.so        /vendor/lib64/
adb push adapsdepthsettings.xml   /vendor/etc/camera/
adb push SpadisQT                 /usr/bin/
adb push SpadisQT_console         /usr/bin/

# 3) Make executable (on the board)
chmod +x /usr/bin/SpadisQT /usr/bin/SpadisQT_console
```

Resulting on-board layout (see the [deployment diagram](../vx_images/deployment.png)):

| Path | Content |
|------|---------|
| `/usr/bin/SpadisQT`, `SpadisQT_console` | the two executables |
| `/vendor/lib64/libadaps_swift_decode.so`, `libAdapsSender.so` | algo + transport libs (binary links with `-rpath /vendor/lib64`, so it finds them here) |
| `/vendor/etc/camera/adapsdepthsettings.xml` | algorithm config |
| `/data/vendor/camera/` | runtime dump directory |

> If `/vendor` is read-only, `mount -o remount,rw /vendor` first, or place the libs in a writable rootfs location and adjust accordingly.

---

## 8. Step 6 — Run and verify

```bash
# Headless build (easiest to bring up first — use it to validate the pipeline)
SpadisQT_console

# GUI build (needs a display + display backend)
SpadisQT
```

### What happens at startup
1. `QLockFile` enforces **single-instance** operation (a second launch exits immediately);
2. `requirement_check()` checks that `/vendor/lib64/libadaps_swift_decode.so` and `/vendor/etc/camera/adapsdepthsettings.xml` exist — and names whichever is missing;
3. It opens the V4L2 device, starts streaming, and the `FrameProcessThread` worker begins capturing and decoding.

### GUI shows nothing?
The GUI needs a Qt display backend. If the board has no X11/Wayland, try EGLFS or LinuxFB:

```bash
QT_QPA_PLATFORM=eglfs SpadisQT
# or
QT_QPA_PLATFORM=linuxfb SpadisQT
```

### Enable verbose logging
The app prints only key logs by default. For detail, enable `debug_info_enable`:

```bash
debug_info_enable=true SpadisQT_console
```

### Quickly verify capture
```bash
# Dump a few raw + depth frames to /data/vendor/camera to check the pipeline
save_frame_raw_data_enable=true save_frame_depth16_enable=true SpadisQT_console
ls -l /data/vendor/camera/
```

✅ Seeing frame-rate logs and data written to `/data/vendor/camera/` means the capture + decode pipeline is working.

---

## 9. Porting to a new board / SoC

Work through these in order:

1. **Kernel driver.** First integrate and bring up the `ads6401` driver in the new board's kernel (device tree: MIPI, I2C, reset/power GPIOs), and confirm `/dev/video*`, `/dev/media*`, `/dev/ads6401` appear. This is outside this repo's scope — get it from the matching kernel SDK.
2. **Device-node paths.** If the nodes aren't the default `/dev/video0` + `/dev/media0`, edit `common.h`:
   - RK3568 branch: `MEDIA_DEVNAME_4_DTOF_SENSOR` / `VIDEO_DEV_4_DTOF_SENSOR`;
   - RK3588 uses the `#else` branch (`/dev/media2` + `/dev/video22`) and enables `VIDIOC_S_FMT_INCLUDE_VIDIOC_SUBDEV_S_FMT` (on RK3588, `VIDIOC_S_FMT` also sets the sensor format);
   - Platform is selected by whether `RUN_ON_RK3568` is defined (in the `.pro`).
3. **Algorithm config.** Per [Section 6](#6-step-4--configure-adapsdepthsettingsxml), set `BufferWidthPHR/FHR` to the new SoC's values and pick the right `ConvolutionType` (NEON=1 on ARM).
4. **Prebuilt libraries.** Confirm `libadaps_swift_decode.so` and `libAdapsSender.so` are ABI-compatible with the new platform (both aarch64). If the algo lib version changes, sync the `ALGO_LIB_VERSION_*` macros in `depthmapwrapper.h` — otherwise version-gated code won't compile in.
5. **Qt backend.** Confirm the new board's Qt display backend (GUI build).
6. **Rebuild + deploy + verify.** Back to Sections 5–8.

> Old-algo compatibility: to pair with an older `libadaps_swift_decode.so`, enable `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB` in the `.pro`. The repo also keeps `libadaps_swift_decode_3.3.2.so` for reference/rollback.

---

## 10. Host communication (optional)

To stream frames to a PC tool (or let the PC send commands / download calibration data), you use `Host_Communication` + `libAdapsSender.so`.

- It is a **singleton** that receives host commands via callback: start/stop capture, read/write registers, update EEPROM, download walk-error / spot-offset / reference-distance / lens-intrinsic / ROI-SRAM calibration data, set the color-map range, reboot, set RTC, etc.;
- In the other direction it uploads raw / depth16 / point-cloud / histogram frames to the host.

If your product needs **no host at all**, define `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` in the `.pro` — the whole communication layer (including `host_comm.cpp`) is excluded from the build.

> ⚠️ Note: `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` changes the **signature** of `rx_new_frame` / `new_frame_handle` (with/without the `frame_buffer_param_t` argument). When editing those signals/slots, update both branches.

---

## 11. Environment-variable reference

All variables are defined in `common.h` (prefix `ENV_VAR_*`). Set them to `true` (or a value) at runtime — no rebuild needed. Common ones by category:

**Data dump / replay**
| Variable | Effect |
|----------|--------|
| `save_frame_raw_data_enable` | dump raw frames |
| `save_frame_depth16_enable` | dump depth16 data |
| `save_frame_pointcloud_enable` | dump point-cloud data |
| `save_frame_histogram_data_enable` | dump histogram data |
| `save_depth_txt_enable` | dump depth as text |
| `save_eeprom_enable` | dump EEPROM data |
| `raw_file_replay_enable` | replay a local raw file (debug without the sensor) |
| `depth16_file_replay_enable` | replay a local depth16 file |
| `enable_algo_lib_dump_data` | let the algo lib dump intermediate data to `/tmp/OfflineData/` |

**Image processing toggles**
| Variable | Effect |
|----------|--------|
| `mirror_x_enable` / `mirror_y_enable` | horizontal / vertical mirror |
| `enable_expand_pixel` | pixel expansion (in algo lib) |
| `disable_compose_subframe` | disable sub-frame composition |
| `disable_walk_error` | disable walk-error compensation |

**Forced parameters** (override defaults, for debugging)
| Variable | Effect |
|----------|--------|
| `force_framerate_fps` | force frame rate |
| `force_poll_timeout` | force poll timeout (ms) |
| `force_coarseExposure` / `force_fineExposure` / `force_grayExposure` | force exposure values |
| `force_laserExposurePeriod` | force laser exposure period |
| `force_row_search_range` / `force_column_search_range` | force SPAD search range |
| `force_frame_buffer_count` | force frame buffer count |
| `force_get_histogram_count` | force histogram count to fetch |

**Logging / tracing**
| Variable | Effect |
|----------|--------|
| `debug_info_enable` | enable verbose `DBG_INFO` logging |
| `trace_algo_lib_decode_costtime` | trace algo decode time |
| `trace_output_frame_rate` | trace output frame rate |
| `trace_roi_sram_switch` | trace ROI-SRAM switching |
| `frame_drop_check_enable` | frame-drop detection |
| `skip_frame_decode` / `skip_frame_process` | skip decode / processing (perf baseline) |

> There is also a family of `dump_*` variables (e.g. `dump_module_static_data`, `dump_eeprom_data`, `dump_capture_req_param`, `dump_depth_map_frame_interval`) for one-shot dumps of various debug info. Full list in `common.h`.

---

## 12. Compile-time define reference

Set via `DEFINES +=` in the `.pro` files; each gates a whole feature block.

| Define | Effect |
|--------|--------|
| `RUN_ON_EMBEDDED_LINUX` | **Master switch.** Only with it are `adaps_dtof.cpp`, `host_comm.cpp`, `misc_device.cpp` compiled in and the two `.so` linked. Without it, only the V4L2/RGB skeleton builds |
| `RUN_ON_RK3568` | use RK3568 device nodes; undefined → RK3588 branch |
| `CONFIG_VIDEO_ADS6401` | enable ADS6401-specific code |
| `ENABLE_POINTCLOUD_OUTPUT` | enable point-cloud output (also gated on algo lib ≥ 3.5.6) |
| `CONSOLE_APP_WITHOUT_GUI` | headless build; `MainWindow` becomes a `QObject`, GUI/QImage code excluded (set only by `SpadisQT_console.pro`) |
| `STANDALONE_APP_WITHOUT_HOST_COMMUNICATION` | drop the host-communication layer (changes `rx_new_frame` signature) |
| `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB` | compatibility with older algo lib |

> Also: `VERSION_*` in `common.h` is the app version (e.g. `3.6.7_LM20260402A`); `ALGO_LIB_VERSION_*` in `depthmapwrapper.h` is the algo-lib version. **Sync the latter when swapping the `.so`**, or version-`#if`-gated code will mis-compile.

---

## 13. Troubleshooting

| Symptom | Likely cause / check |
|---------|----------------------|
| Startup error `Adaps algo lib <...> does not exist` | `libadaps_swift_decode.so` not in `/vendor/lib64/`, or wrong path |
| Startup error `config file <...> does not exist` | `adapsdepthsettings.xml` not in `/vendor/etc/camera/` |
| Runtime `cannot open shared object` | lib not deployed, or `/vendor/lib64` read-only; confirm `-rpath` points to `/vendor/lib64` |
| App runs but **no frames / poll timeout** | back to [Section 3](#3-step-1--verify-the-sensor): do `/dev/video0`, `/dev/ads6401` exist, does `media-ctl` show `ads6401`; is the sensor connected and powered |
| Sensor subdevice not found | `KEYWORD_4_DTOF_SENSOR_SUBDEV_NAME` (`ads6401`) in `common.h` doesn't match the media-graph name, or wrong node path |
| **Corrupted image / clearly wrong depth** | `BufferWidthPHR/FHR` in `adapsdepthsettings.xml` doesn't match the SoC (RK should be `1280/4352`); or wrong `ConvolutionType` |
| GUI won't start / black screen | missing display backend — try `QT_QPA_PLATFORM=eglfs` or `linuxfb`; confirm the board has a display |
| Low frame rate | use `trace_algo_lib_decode_costtime`, `trace_output_frame_rate` to locate the bottleneck; confirm `ConvolutionType=1` (NEON) |
| Second launch exits immediately | single-instance protection (`QLockFile`) — expected; kill the existing process first |
| App crashed, want a stack trace | build the **Debug** target (keeps `-rdynamic`); the signal handler in `main.cpp` prints a backtrace with function names |
| Unsure if it's the sensor or the algorithm | use `raw_file_replay_enable` to replay a known-good raw file and rule out the sensor |

---

## Still stuck?

- Read the [Ads6401 dToF SDK for Linux User Guide (PDF)](Ads6401_dToF_SDK_For_Linux_User_Guide.pdf);
- Contact [ADAPS Photonics](https://adapsphotonics.com/).

> Basic environment questions such as "does my board support Qt" are out of scope for this project — please research those yourself.
