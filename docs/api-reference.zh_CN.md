# SpadisQT — API 文档

[English](api-reference.md) · [文档索引](README.zh_CN.md)

本参考涵盖**内部 C++ 类 API**、关键共享类型，以及应用所依赖的三套外部契约：解码库
（`depthmapwrapper.h`）、上位机协议（`host_device_comm_types.h`）、`ads6401` 内核 ioctl
（`adaps_dtof_uapi.h`）。下文所有签名均逐字取自头文件。

> 预编译 `.so` 为闭源。`depthmapwrapper.h`、`adaps_sender.h`、`host_device_comm_types.h`、
> `adaps_dtof_uapi.h` 是**手工维护的契约**——其中之一变更时，对应 `.so`（或内核驱动）必须
> 同步变更。

---

## 1. `V4L2`（`v4l2.h`）

管理一个 V4L2 采集设备。以 `sensor_params` 构造。

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

**信号**

```cpp
// 含上位机通信（默认）：
void rx_new_frame(unsigned int frm_sequence, void *frm_buf, int frm_len,
                  struct timeval frm_timestamp, enum frame_data_type ftype,
                  int total_bytes_per_line, frame_buffer_param_t frmBufParam);
// 定义 STANDALONE_APP_WITHOUT_HOST_COMMUNICATION 时：去掉最后一个参数。
void update_info(status_params1 param1);
```

---

## 2. `FrameProcessThread`（`FrameProcessThread.h`）

采集+解码工作线程 `QThread`。

```cpp
void stop(int stop_request_code);     // 取值见 enum stop_request_code
int  init(int index);
bool isSleeping();
void setWatchSpot(QSize widget_size, QPoint point);  // 仅 GUI 构建
protected: void run();                // 采集循环
```

**槽** —— `new_frame_handle(...)`（两种签名，取决于上位机通信宏）、
`info_update(status_params1)`、`onThreadLoopExit()`。

**信号** —— `newFrameReady4Display(uint, QImage, QImage)`（仅 GUI）、
`update_runtime_display(status_params2)`、`frame_rx_error(int)`、`threadLoopExit()`、
`threadEnd(int)`。

---

## 3. `ADAPS_DTOF`（`adaps_dtof.h`）

封装解码库。以 `sensor_params` 构造。

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

`dtof_frame_decode()` 是核心调用：输入原始字节，输出 depth16 / 点云 / 直方图缓冲区。输出
格式见 [数据流程 §3](data-flow.zh_CN.md#3-depth16-格式)。

---

## 4. `Misc_Device`（`misc_device.h`，需 `RUN_ON_EMBEDDED_LINUX`）

对 `/dev/ads6401` 的全部 ioctl。

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

`get_dtof_module_static_data()` 报告模组类型（Spot / Flood / Big-FoV Flood /
Big-FoV Flood V2）。外部配置脚本在写入驱动前会被解析为 `ScriptItem`
（`I2C_Write`/`I2C_Read`/`MIPI_Read`/`Swift_Delay`/`Block_*`/`Slave_Addr`）。

---

## 5. `Host_Communication`（`host_comm.h`，需含上位机通信的构建）

**单例** —— 经 `Host_Communication::getInstance()` 获取；拷贝/赋值已删除。

```cpp
static Host_Communication* getInstance();

int upload_frame_raw_data      (void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_frame_depth16_data  (void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_frame_pointcloud_data(void* p, uint32_t size, frame_buffer_param_t* bp);
int upload_req_histogram_data  (void* p, uint32_t size, frame_buffer_param_t* bp);
int report_status(UINT16 responsed_cmd, UINT16 status_code, char* msg, int msg_length);

// 由上位机命令置位的请求状态读取器：
UINT8   get_req_out_data_type();        BOOLEAN get_req_compose_subframe();
UINT8   get_req_walkerror_enable();     BOOLEAN get_req_expand_pixel();
BOOLEAN get_req_mirror_x();             BOOLEAN get_req_mirror_y();
void*   get_loaded_ref_distance_data(); void*   get_loaded_lens_intrinsic_data();
uint16_t get_req_histogram_x();         uint16_t get_req_histogram_y();
```

**信号** —— `start_capture()`、`stop_capture()`、`set_capture_options(capture_req_param_t*)`。

### 上位机 ↔ 设备命令集（`host_device_comm_types.h`）

来自**上位机**的命令（`CMD_HOST_SIDE_*`）：

| 命令 | 用途 |
|------|------|
| `SET_COLORMAP_RANGE_PARAM` | 设置深度→颜色映射范围 |
| `GET_MODULE_STATIC_DATA` | 获取 EEPROM/OTP 标定 + 模组类型 |
| `DOWNLOAD_ROI_SRAM_DATA` | 向设备下发 ROI-SRAM 数据 |
| `START_CAPTURE` / `STOP_CAPTURE` | 启动 / 停止采集 |
| `SET_SENSOR_REGISTER` / `GET_SENSOR_REGISTER` | 传感器寄存器读写 |
| `SET_VCSLDRV_OP7020_REGISTER` / `GET_…` | VCSEL 驱动寄存器读写 |
| `DOWNLOAD_SPOT_WALKERROR_DATA` | 下发 walk-error 标定 |
| `DOWNLOAD_SPOT_OFFSET_DATA` | 下发光斑 offset 标定 |
| `SET_WALKERROR_ENABLE` | 开关 walk-error 校正 |
| `UPDATE_EEPROM_DATA` | 写 EEPROM |
| `SET_DEVICE_REBOOT` | 重启设备 |
| `SET_RTC_TIME` | 设置 RTC |
| `SET_EXPOSURE_TIME` | 设置曝光 |
| `DOWNLOAD_REF_DISTANCE_DATA` | 下发参考距离标定 |
| `DOWNLOAD_LENS_INTRINSIC_DATA` | 下发镜头内参标定 |
| `SET_HISTOGRAM_DATA_REQ_POSITION` | 选择要推流直方图的光斑位置 |
| `SET_ADAPS_ALGO_MODEL_TYPE` | 选择算法模型类型 |

来自**设备**的回复（`CMD_DEVICE_SIDE_*`）：`UPLOAD_MODULE_STATIC_DATA`、
`UPLOAD_FRAME_RAW_DATA`、`UPLOAD_FRAME_DEPTH16_DATA`、`UPLOAD_FRAME_POINTCLOUD_DATA`、
`UPLOAD_REQ_POS_HISTOGRAM_DATA`、`REPORT_STATUS`、`REPORT_SENSOR_REGISTER`、
`REPORT_VCSLDRV_OP7020_REGISTER`。状态码：`NO_ERROR`、`ERROR_INVALID_PARAM`、
`ERROR_MISMATCHED_WORK_MODE`、`ERROR_WRITE_REGISTER` 等。

> 注意：头文件中命令操作码存在**两套编号**（一套遗留、一套现行）。务必使用符号化的
> `CMD_*` 名称，切勿使用裸数字。

---

## 6. 解码库契约（`depthmapwrapper.h`）

闭源 `libadaps_swift_decode.so` 暴露一套 C ABI：

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

关键类型：`WrapperDepthInitInputParams`、`SetWrapperParam`（标定与各模组配置）、
`WrapperDepthInput`、`WrapperDepthCamConfig`、`WrapperDepthOutput`、`pc_pkt_t`
（`{float X,Y,Z,c;}`）、`SpotPoint`、`SpotHistogram`。枚举：`AdapsModelType`、
`WrapperDepthFormat`、`RotateConfig`、`WrapperDepthExpModeType`。

### 版本管理

```c
#define ALGO_LIB_VERSION_MAJOR     3
#define ALGO_LIB_VERSION_MINOR     8
#define ALGO_LIB_VERSION_REVISION  5
```

结构体布局受版本门控（`#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(...)`），阈值为
**3.5.6**（点云输出字段）、**3.6.2**（`dump_data`/`save_path`）、**3.6.5**
（`adapsAlgoModelType`）。仓库保留了回滚参考二进制 `libadaps_swift_decode_3.3.2.so`；
选用它需要 `ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB`。

---

## 7. 运行期环境变量（`common.h`）

布尔开关取字面字符串 `"true"`（`env_var_is_true()`）；`force_*` 类取具体值。重点节选
（完整 `ENV_VAR_*` 列表见 `common.h`）：

**保存 / 转储** —— `save_eeprom_enable`、`save_depth_txt_enable`、
`save_frame_raw_data_enable`、`save_frame_depth16_enable`、`save_frame_pointcloud_enable`、
`save_frame_histogram_data_enable`、`enable_algo_lib_dump_data`，以及庞大的 `dump_*` 系列
（`dump_lens_intrinsic`、`dump_module_static_data`、`dump_eeprom_data` 等）。

**流水线控制** —— `skip_frame_decode`、`skip_frame_process`、`enable_expand_pixel`、
`disable_compose_subframe`、`disable_walk_error`、`mirror_x_enable`、`mirror_y_enable`。

**回放** —— `raw_file_replay_enable`、`depth16_file_replay_enable`、
`save_loaded_data_enable`。

**强制参数** —— `force_framerate_fps`、`force_poll_timeout`、`force_frame_buffer_count`、
`force_coarseExposure`、`force_fineExposure`、`force_grayExposure`、
`force_laserExposurePeriod`、`force_row_search_range`、`force_column_search_range`、
`force_roi_sram_size`、`force_frame_process`、`force_get_histogram_count`。

**追踪 / 调试** —— `debug_info_enable`（控制所有 `DBG_INFO` 输出）、
`trace_algo_lib_decode_costtime`、`trace_output_frame_rate`、`trace_roi_sram_switch`。

> 优先新增 `ENV_VAR_*` 开关，而非硬编码调试行为。新代码请通过
> `DBG_ERROR` / `DBG_NOTICE` / `DBG_INFO` / `DBG_PRINTK`（带时间戳、经 `qCritical` 路由）
> 输出日志——切勿使用裸 `printf`/`qDebug`。

---

## 8. `GlobalApplication`（`globalapplication.h`）

全局注册表，经重载的 `qApp` 宏访问：

```cpp
#define qApp (static_cast<GlobalApplication *>(QCoreApplication::instance()))
```

提供以下内容的 getter/setter：已选传感器类型与工作模式、模组类型、功耗 / 帧率 / 环境 /
测量类型、颜色映射范围（`get/set_RealDistanceMin/MaxMappedRange`、
`get/set_GrayScaleMin/MaxMappedRange`）、已加载的标定缓冲（walk-error、光斑 offset、
ROI-SRAM、镜头内参、参考距离）、曝光值、光斑搜索范围，以及已注册的 `V4L2`/`Misc_Device`
单例。

---

## 9. 共享类型（`common.h`）

`enum sensortype`、`enum sensor_workmode`、`enum frame_data_type`、`enum stop_request_code`、
`enum frame_rx_err_type`；`struct sensor_params`（传给 `V4L2`/`ADAPS_DTOF` 的逐次采集配置）、
`struct status_params1` / `status_params2`（上报给 UI 的运行期状态）。后两者经
`Q_DECLARE_METATYPE` 声明，从而可经队列信号跨线程传递。
