# SpadisQT — API Reference

[简体中文](api-reference.zh_CN.md) · [Docs index](README.md)

This reference covers the **internal C++ class APIs**, the key shared types, and the three
external contracts the app is built against: the decode library (`depthmapwrapper.h`), the
host protocol (`host_device_comm_types.h`), and the `ads6401` kernel ioctls
(`adaps_dtof_uapi.h`). All signatures below are taken verbatim from the headers.

> The prebuilt `.so` files are closed-source. `depthmapwrapper.h`, `adaps_sender.h`,
> `host_device_comm_types.h` and `adaps_dtof_uapi.h` are the **hand-maintained contracts** —
> when one changes, the matching `.so` (or kernel driver) must change in lockstep.

---

## 1. `V4L2` (`v4l2.h`)

Owns one V4L2 capture device. Constructed with a `sensor_params`.

```cpp
int  V4l2_initilize(void);
int  Start_streaming(void);
int  Capture_frame();
void Stop_streaming(void);
void V4l2_close(void);
void Get_frame_size_4_curr_wkmode(int *in_w, int *in_h, int *out_w, int *out_h);
bool Get_power_on_state();
bool Get_stream_on_state();
int  Get_videodev_fd();
```

**Signals**

```cpp
// With host comm (default):
void rx_new_frame(unsigned int frm_sequence, void *frm_buf, int frm_len,
                  struct timeval frm_timestamp, enum frame_data_type ftype,
                  int total_bytes_per_line, frame_buffer_param_t frmBufParam);
// With STANDALONE_APP_WITHOUT_HOST_COMMUNICATION: same minus the last arg.
void update_info(status_params1 param1);
```

---

## 2. `FrameProcessThread` (`FrameProcessThread.h`)

The capture+decode worker `QThread`.

```cpp
void stop(int stop_request_code);     // codes from enum stop_request_code
int  init(int index);
bool isSleeping();
void setWatchSpot(QSize widget_size, QPoint point);  // GUI builds only
protected: void run();                // the capture loop
```

**Slots** — `new_frame_handle(...)` (two signatures, see the host-comm flag),
`info_update(status_params1)`, `onThreadLoopExit()`.

**Signals** — `newFrameReady4Display(uint, QImage, QImage)` (GUI only),
`update_runtime_display(status_params2)`, `frame_rx_error(int)`, `threadLoopExit()`,
`threadEnd(int)`.

---

## 3. `ADAPS_DTOF` (`adaps_dtof.h`)

Wraps the decode library. Constructed with `sensor_params`.

```cpp
int  adaps_dtof_initilize();
int  dtof_frame_decode(enum sensor_workmode swk, unsigned int frm_sequence,
                       unsigned char *frm_rawdata, int frm_rawdata_size,
                       u16 depth16_buffer[], pc_pkt_t *point_cloud_buffer,
                       struct SpotHistogram *hist_buf, int *HistCountToGet);
void ConvertDepthToColoredMap(const u16 depth16_buffer[], u8 depth_colored_map[],
                              u8 depth_confidence_map[], int outW, int outH);
void ConvertGreyscaleToColoredMap(u16 depth16_buffer[], u8 colored_map[], int outW, int outH);
void GetDepth4watchSpot(const u16 depth16_buffer[], int outW, u8 x, u8 y,
                        u16 *distance, u8 *confidence);
SpotPoint* get_spcific_histogram(uint16_t x, uint16_t y);
void adaps_dtof_release();
```

`dtof_frame_decode()` is the central call: raw bytes in, depth16 / point-cloud / histogram
buffers out. See [Data Flow §3](data-flow.md#3-the-depth16-format) for the output format.

---

## 4. `Misc_Device` (`misc_device.h`, requires `RUN_ON_EMBEDDED_LINUX`)

All ioctl traffic to `/dev/ads6401`.

```cpp
int   read_dtof_runtime_status_param(struct adaps_dtof_runtime_status_param **status_param);
int   get_dtof_inside_temperature(float *temperature);
void* get_dtof_calib_eeprom_param(void);
void* get_dtof_exposure_param(void);
void* get_dtof_runtime_status_param(void);
int   write_device_register(register_op_data_t *reg);
int   read_device_register(register_op_data_t *reg);
int   read_dtof_exposure_param(void);
int   write_dtof_initial_param(struct adaps_dtof_intial_param *param);
int   get_dtof_module_static_data(void **pp_module_static_data,
                                  void **pp_eeprom_data_buffer, uint32_t *eeprom_data_size);
int   send_down_loaded_roisram_data_size(const uint32_t roi_sram_size);
int   send_down_external_config(UINT8 workMode, uint32_t script_buf_size, const uint8_t* script_buf);
int   update_eeprom_data(UINT8 *buf, UINT32 offset, UINT32 length);
```

`get_dtof_module_static_data()` reports the module type (Spot / Flood / Big-FoV Flood /
Big-FoV Flood V2). The external config script is parsed into `ScriptItem`s
(`I2C_Write`/`I2C_Read`/`MIPI_Read`/`Swift_Delay`/`Block_*`/`Slave_Addr`) before being
written to the driver.

---

## 5. `Host_Communication` (`host_comm.h`, requires host-comm build)

**Singleton** — obtain via `Host_Communication::getInstance()`; copy/assignment deleted.

```cpp
static Host_Communication* getInstance();

int upload_frame_raw_data      (void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_frame_depth16_data  (void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_frame_pointcloud_data(void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_req_histogram_data  (void* p, uint32_t size, frame_buffer_param_t* bp);
int report_status(UINT16 responsed_cmd, UINT16 status_code, char* msg, int msg_length);

// request-state getters set by host commands:
UINT8   get_req_out_data_type();        BOOLEAN get_req_compose_subframe();
UINT8   get_req_walkerror_enable();     BOOLEAN get_req_expand_pixel();
BOOLEAN get_req_mirror_x();             BOOLEAN get_req_mirror_y();
void*   get_loaded_ref_distance_data(); void*   get_loaded_lens_intrinsic_data();
uint16_t get_req_histogram_x();         uint16_t get_req_histogram_y();
```

**Signals** — `start_capture()`, `stop_capture()`, `set_capture_options(capture_req_param_t*)`.

### Host ↔ device command set (`host_device_comm_types.h`)

Commands **from the host** (`CMD_HOST_SIDE_*`):

| Cmd | Purpose |
|-----|---------|
| `SET_COLORMAP_RANGE_PARAM` | Set the depth→color mapping range |
| `GET_MODULE_STATIC_DATA` | Fetch EEPROM/OTP calibration + module type |
| `DOWNLOAD_ROI_SRAM_DATA` | Download ROI-SRAM data to the device |
| `START_CAPTURE` / `STOP_CAPTURE` | Start / stop capture |
| `SET_SENSOR_REGISTER` / `GET_SENSOR_REGISTER` | Sensor register R/W |
| `SET_VCSLDRV_OP7020_REGISTER` / `GET_…` | VCSEL-driver register R/W |
| `DOWNLOAD_SPOT_WALKERROR_DATA` | Download walk-error calibration |
| `DOWNLOAD_SPOT_OFFSET_DATA` | Download spot-offset calibration |
| `SET_WALKERROR_ENABLE` | Toggle walk-error correction |
| `UPDATE_EEPROM_DATA` | Write EEPROM |
| `SET_DEVICE_REBOOT` | Reboot the device |
| `SET_RTC_TIME` | Set the RTC |
| `SET_EXPOSURE_TIME` | Set exposure |
| `DOWNLOAD_REF_DISTANCE_DATA` | Download reference-distance calibration |
| `DOWNLOAD_LENS_INTRINSIC_DATA` | Download lens-intrinsic calibration |
| `SET_HISTOGRAM_DATA_REQ_POSITION` | Select the spot whose histogram to stream |
| `SET_ADAPS_ALGO_MODEL_TYPE` | Select the algo model type |

Replies **from the device** (`CMD_DEVICE_SIDE_*`): `UPLOAD_MODULE_STATIC_DATA`,
`UPLOAD_FRAME_RAW_DATA`, `UPLOAD_FRAME_DEPTH16_DATA`, `UPLOAD_FRAME_POINTCLOUD_DATA`,
`UPLOAD_REQ_POS_HISTOGRAM_DATA`, `REPORT_STATUS`, `REPORT_SENSOR_REGISTER`,
`REPORT_VCSLDRV_OP7020_REGISTER`. Status codes: `NO_ERROR`, `ERROR_INVALID_PARAM`,
`ERROR_MISMATCHED_WORK_MODE`, `ERROR_WRITE_REGISTER`, …

> Note: the command opcodes have **two numbering schemes** in the header (a legacy block and
> the current block). Always use the symbolic `CMD_*` names, never the raw numbers.

---

## 6. Decode-library contract (`depthmapwrapper.h`)

The closed-source `libadaps_swift_decode.so` exposes a C ABI:

```cpp
int  DepthMapWrapperCreate(void** handler,
                           WrapperDepthInitInputParams  in,
                           WrapperDepthInitOutputParams out);
bool DepthMapWrapperProcessFrame(void* handler, WrapperDepthInput in_image,
                                 WrapperDepthCamConfig* cfg,
                                 uint32_t num_outputs, WrapperDepthOutput outputs[]);
void DepthMapWrapperDestroy(void* handler);
void DepthMapWrapperGetVersion(char* version);
void DepthMapWrapperSetCircleMask(void* h, CircleForMask m);
void DepthMapWrapperGetDataInfo(void* h, uint32_t* sramId, uint32_t* zoneId);
```

Key types: `WrapperDepthInitInputParams`, `SetWrapperParam` (calibration & per-module
config), `WrapperDepthInput`, `WrapperDepthCamConfig`, `WrapperDepthOutput`, `pc_pkt_t`
(`{float X,Y,Z,c;}`), `SpotPoint`, `SpotHistogram`. Enums: `AdapsModelType`,
`WrapperDepthFormat`, `RotateConfig`, `WrapperDepthExpModeType`.

### Versioning

```c
#define ALGO_LIB_VERSION_MAJOR     3
#define ALGO_LIB_VERSION_MINOR     8
#define ALGO_LIB_VERSION_REVISION  5
```

Struct layout is version-gated (`#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(...)`) at
**3.5.6** (point-cloud output fields), **3.6.2** (`dump_data`/`save_path`) and **3.6.5**
(`adapsAlgoModelType`). A reference rollback blob `libadaps_swift_decode_3.3.2.so` is kept;
selecting it requires `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB`.

---

## 7. Runtime environment variables (`common.h`)

Boolean toggles take the literal string `"true"` (`env_var_is_true()`); the `force_*` ones
take a value. Selected highlights (see `common.h` for the full `ENV_VAR_*` list):

**Save / dump** — `save_eeprom_enable`, `save_depth_txt_enable`,
`save_frame_raw_data_enable`, `save_frame_depth16_enable`, `save_frame_pointcloud_enable`,
`save_frame_histogram_data_enable`, `enable_algo_lib_dump_data`, plus a large `dump_*`
family (`dump_lens_intrinsic`, `dump_module_static_data`, `dump_eeprom_data`, …).

**Pipeline control** — `skip_frame_decode`, `skip_frame_process`, `enable_expand_pixel`,
`disable_compose_subframe`, `disable_walk_error`, `mirror_x_enable`, `mirror_y_enable`.

**Replay** — `raw_file_replay_enable`, `depth16_file_replay_enable`,
`save_loaded_data_enable`.

**Force parameters** — `force_framerate_fps`, `force_poll_timeout`,
`force_frame_buffer_count`, `force_coarseExposure`, `force_fineExposure`,
`force_grayExposure`, `force_laserExposurePeriod`, `force_row_search_range`,
`force_column_search_range`, `force_roi_sram_size`, `force_frame_process`,
`force_get_histogram_count`.

**Tracing / debug** — `debug_info_enable` (gates all `DBG_INFO` output),
`trace_algo_lib_decode_costtime`, `trace_output_frame_rate`, `trace_roi_sram_switch`.

> Prefer adding a new `ENV_VAR_*` toggle over hard-coding debug behavior. Log through
> `DBG_ERROR` / `DBG_NOTICE` / `DBG_INFO` / `DBG_PRINTK` (timestamped, routed via
> `qCritical`) — never bare `printf`/`qDebug` in new code.

---

## 8. `GlobalApplication` (`globalapplication.h`)

Global registry, reached via the overridden `qApp` macro:

```cpp
#define qApp (static_cast<GlobalApplication *>(QCoreApplication::instance()))
```

Provides getters/setters for the selected sensor type & work mode, module type, power /
framerate / environment / measurement types, color-map ranges
(`get/set_RealDistanceMin/MaxMappedRange`, `get/set_GrayScaleMin/MaxMappedRange`), the loaded
calibration buffers (walk-error, spot-offset, ROI-SRAM, lens-intrinsic, ref-distance),
exposure values, spot search ranges, and the registered `V4L2`/`Misc_Device` singletons.

---

## 9. Shared types (`common.h`)

`enum sensortype`, `enum sensor_workmode`, `enum frame_data_type`, `enum stop_request_code`,
`enum frame_rx_err_type`; `struct sensor_params` (the per-capture configuration passed to
`V4L2`/`ADAPS_DTOF`), `struct status_params1` / `status_params2` (runtime status reported to
the UI). The latter are `Q_DECLARE_METATYPE`'d so they can cross thread boundaries via
queued signals.
