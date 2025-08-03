/**
 * @file adaps_sender.h
 * @author rongchao (rongchao.xu@adaps-ph.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __ADAPS_SENDER_H__
#define __ADAPS_SENDER_H__
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>

#ifdef _MSC_VER
#ifdef SENDER_API_EXPORTS
#define SENDER_API __declspec(dllexport)
#else
#define SENDER_API __declspec(dllimport)
#endif
#else
#define SENDER_API extern
#endif

#define SENDER_VERSION_MAJOR        1
#define SENDER_VERSION_MINOR        6
#define SENDER_VERSION_REVISION     0

#define TCP_SERVER_PORT 8765

#define SENDER_TCP_IP_LEN 20

#define SENDER_QBUF_CNT_NOT_LIMIT       0
#define SENDER_QBUF_CNT_LIMIT_DEFAULT   SENDER_QBUF_CNT_NOT_LIMIT

#define SENDER_QBUF_TOTAL_SIZE_NOT_LIMIT  0
#define SENDER_QBUF_TOTAL_SIZE_LIMIT_DEFAULT  (500 * 1024 * 1024) //500M

#define SENDER_RUN_AS_HOST    0
#define SENDER_RUN_AS_DEVICE  1

/**
 * @brief sender日志输出函数指针类型定义，函数参数与返回值与printf一致
 * 
 */
typedef int (*sender_log_output_func_t)(const char *fmt, ...);

/**
 * @brief sender日志前缀字符串获取函数指针类型定义
 * 
 * @param buf 日志前缀字符串buffer指针
 * @param size 参数buf的空间大小
 */
typedef void (*sender_log_prefix_func_t)(char *buf, int size);

enum SenderLogLevel_e {
    SENDER_LOG_TRACE = 0,
    SENDER_LOG_ERR,
    SENDER_LOG_WARN,
    SENDER_LOG_INFO,
    SENDER_LOG_DBG
};

enum SenderChannel_e {
    SENDER_CHN_NETWORK = 0, //ethernet or rndis
    SENDER_CHN_USB,         //usb ffs
};

enum SenderUsbSpeed_e {
    SENDER_USB_SPEED_UNKNOWN = 0,
    SENDER_USB_SPEED_SUPER,  //usb3.x
    SENDER_USB_SPEED_HIGH,   //usb2.0
    SENDER_USB_SPEED_LOW    //usb1.x
};

enum SenderMsgFlag_e {
    SENDER_DO_CHECKSUM = 1 << 0,
    SENDER_NOTIFY_STATE = 1 << 8, //notify send done/fail through sender_callback_t
};

enum SenderLinkModeMask_e{
    SENDER_LM_CHANNEL = 0xFF,
    SENDER_LM_SET_CHANNEL = (1 << 8),
    SENDER_LM_RNDIS_DISALBE = (1 << 9),
    SENDER_LM_USING_MULTI_HOST = (1 << 10),
};

typedef enum SenderEventId_e {

    /**
     * @brief sender通讯选定通道建立时产生此事件，回调函数实参arg_u32为通道类别enum SenderChannel_e
     *
     */
    SENDER_EVT_CONNECTED = 0,

    /**
     * @brief sender通讯选定通道断开时产生此事件, 回调函数实参arg_u32为通道类别enum SenderChannel_e
     *
     */
    SENDER_EVT_DISCONNECTED = 1,

    /**
     * @brief
     * sender收到对方消息数据时产生此事件，回调函数实参arg_ptr为数据buffer指针，实参arg_u32为数据长度 ;
     *
     */
    SENDER_EVT_RECEIVED_MSG = 2,

    /**
     * @brief
     * sender把异步消息发送出去后产生此事件，回调函数实参arg_u32为sender_async_send_msg()函数返回的msg id ;
     *
     */
    SENDER_EVT_SEND_ASYNC_MSG = 3,


     /**
      * @brief sender通道连接(可用)时产生此事件，回调函数实参arg_u32为通道类别enum SenderChannel_e
      *
      */
     SENDER_EVT_CHN_LINKED = 4,

     /**
      * @brief sender通道断开(不可用)时产生此事件, 回调函数实参arg_u32为通道类别enum SenderChannel_e
      *
      */
     SENDER_EVT_CHN_UNLINKED = 5,


} SenderEventId_t;

/**
 * @brief sender事件回调函数定义
 * 
 */
typedef int (*sender_callback_t)(SenderEventId_t id, void *arg_ptr, uint32_t arg_u32, ...);

typedef void* sender_connection_handle_t;
typedef int (*sender_connection_callback_t)(sender_connection_handle_t hdl, SenderEventId_t id, void *arg_ptr, uint32_t arg_u32, ...);

typedef struct sender_connection_param_s {
    /*
    * 网络连接服务器端的ip地址 ;
    */
    char target_ip[SENDER_TCP_IP_LEN];

    /*
    * 网络连接服务器端端口号 ;
    */
    int target_port;

    /*
    * sender事件回调函数，事件定义见sender_connection_callback_t ;
    */
    sender_connection_callback_t callback;

    /*
    * 预留字节，置为0, 添加字段后需减去增加的字节数 ;
    */
    char reserved[32];

}sender_connection_param_t;

typedef struct sender_connection_info_s
{
    sender_connection_handle_t hdl;
    sender_connection_param_t param;

}sender_connection_info_t;


typedef struct sender_init_param_s {
    /*
    * 0==host端，网络连接作为客户端，usb连接作为host端 ;
    * 1==device端，网络连接作为服务端，usb连接作为device端 ;
    */
    int host_or_device;

    /*
    * 网络连接服务器端的ip地址 ;
    */
    char tcp_ip[SENDER_TCP_IP_LEN];

    /*
    * 网络连接服务器端端口号 ;
    */
    int tcp_port;

    /*
    * sender事件回调函数;
    */
    sender_callback_t callback;

    /*
    * 用来判别是否是通过rndis网卡连接, 约定子网位数是24或16,
    * 例子：子网位数为24, ip为aaa.bbb.ccc.ddd时，此值为:
    * (aaa << 24) | (bbb << 16) | (ccc << 8) | 255
    * 特殊地，为0表示采用默认的169.254.206.000/24网段 ;
    */
    unsigned int rndis_net_mask;

    /*
    * 心跳超时时间，单位为ms，为0时表示默配置(10秒)，负数表示不使能心跳 ;
    */
    int heartbeat_timeout;

    /*
    * 异步发送队列中buffer的最多个数, 为0时表示默认配置(不限制)，负数表示不限制 ;
    */
    unsigned int qbuf_cnt_limit;

    /*
    * 异步发送队列中所有buffer的总大小限值, 为0时表示默认配置(500MB)，负数表示不限制 ;
    */
    unsigned int qbuf_total_size_limit;

    /*
    * 一条报文最大字节数，为0时表示默认配置(100M)
    */
    unsigned int msg_size_max;

    /*
    * 连接方式配置，为0时表示默认方式(通过tcp_ip参数判断) ;
    * bit0~bit7: enum SenderChannel_e定义的连接通道 ;
    * bit8:0=通过tcp_ip参数判断，1=bit0~bit7选定的通道;
    * bit9:0=使能rndis，1=禁用rndis ;
    * bit10:0=不采用多host方式，1=采用多host方式 ;
    * 其他bits: 预留，置为0 ;
    * 掩码定义见：enum SenderLinkModeMask_e;
    */
    unsigned int link_mode;

    /*
    * 采用多host方式时的连接数量，一个连接对应一个device
    */
    unsigned int connection_count;

    /*
    * 采用多host方式时的每个连接信息列表
    */
    sender_connection_info_t *conn_info_list;

    /*
    * 预留字节，全置为0, 添加字段后需减去增加的字节数 ;
    */
    char reserved[52];

}sender_init_param_t;


/**
 * @brief 初始化sender
 * 
 * @param param 见sender_init_param_t定义
 * @return int 0=成功，非0=失败
 */
SENDER_API int sender_init(sender_init_param_t *param);

/**
 * @brief 销毁sender
 * 
 * @return int 0=成功，非0=失败
 * 
 */
SENDER_API int sender_destroy(void);

/**
 * @brief 查询sender选定传输通道是否正常
 * 
 * @return int 0=不正常，正数=正常，负数=未确定
 * 
 */
SENDER_API int sender_is_working(void);
SENDER_API int sender_connection_is_working(sender_connection_handle_t hdl);

/**
 * @brief 同步方式发送一条消息包
 * 
 * @param data_buf 消息内容数据buffer指针
 * @param size 消息内容数据长度
 * @return int 0=成功，非0=失败
 * 
 */
SENDER_API int sender_send_msg(void *data_buf, unsigned int size);
SENDER_API int sender_connection_send_msg(sender_connection_handle_t hdl, void *data_buf, unsigned int size);

/**
 * @brief 申请异步发送buffer
 * 
 * @param size 申请buffer的size
 * @return void* NULL=申请失败，非NULL=申请成功
 * 
 */
SENDER_API void *sender_async_buf_alloc(unsigned int size);

/**
 * @brief 释放异步发送buffer
 * 
 * @param async_buf 使用sender_async_buf_alloc()申请到的buffer指针
 * 
 */
SENDER_API void sender_async_buf_free(void *async_buf);

/**
 * @brief
 * 异步方式发送一条消息包，调用此接口后，sender内部会自行释放async_buf，不用再调用sender_async_buf_free()释放
 *
 * @param async_buf 使用sender_async_buf_alloc()申请到的buffer指针
 * @param size 发送消息的长度
 * @param flag 每一位的含义见enum SenderMsgFlag_e定义，未定义的位设为0 
 * @return int 非负数=发送标识ID，负数=失败
 * 
 */
SENDER_API int sender_async_send_msg(void *async_buf, unsigned int size, unsigned int flag);
SENDER_API int sender_connection_async_send_msg(sender_connection_handle_t hdl, void *async_buf, unsigned int size, unsigned int flag);

/**
 * @brief
 * 设置异步发送队列中buffer的最多个数
 *
 * @param cnt 0=无限制，非0=限制的数量
 * @return int 0=成功，非0=失败
 */
SENDER_API int sender_async_set_qbuf_cnt_limit(unsigned int cnt);


/**
 * @brief
 * 设置异步发送队列中所有buffer的总大小限值
 *
 * @param size_kb 0=无限制，非0=限制的总大小(单位：byte)
 * @return int 0=成功，非0=失败
 */
SENDER_API int sender_async_set_qbuf_total_size_limit(unsigned int size);


/**
 * @brief 设置通讯通道
 *
 * @param chn 通道，见enum SenderChannel_e定义
 * @return int 0=成功，非0=失败
 *
 */
SENDER_API int sender_channel_set_sel(enum SenderChannel_e chn);


/**
 * @brief 获取当前选定通讯通道
 *
 * @return int 负数=通道未选定或通讯双方在握手协定中，其他=当前选定的通道，见enum SenderChannel_e定义
 */
SENDER_API int sender_channel_get_sel(void);

/**
 * @brief 获取通道连接状态
 *
 * @param chn 通道，见enum SenderChannel_e定义
 * @return int 负数=未确定，0=断开，正数=已连接
 *
 */
SENDER_API int sender_channel_is_linked(enum SenderChannel_e chn);

/**
 * @brief 获取rndis连接状态
 *
 * @param chn 通道，见enum SenderChannel_e定义
 * @return int 负数=未确定，0=断开，正数=已连接
 *
 */
SENDER_API int sender_rndis_is_linked(void);

/**
 * @brief 获取usb连接速率
 *
 * @return enum SenderUsbSpeed_e
 *
 */
SENDER_API enum SenderUsbSpeed_e sender_usb_speed_get(void);

/**
 * @brief 配置sender日志等级及日志函数
 * 
 * @param level sender日志等级, 见enum SenderLogLevel_e定义
 * @param func 日志输出函数指针
 *
 */
SENDER_API void sender_log_config(enum SenderLogLevel_e level, sender_log_output_func_t func);

/**
 * @brief 设置sender日志前缀获取函数，每条日志的前缀都会加上此函数返回的字符串。若没设置则日志带时间戳前缀
 * 
 * @param func 日志前缀获取函数指针
 *
 */
SENDER_API void sender_set_log_prefix_func(sender_log_prefix_func_t func);

/**
 * @brief 获取sender版本字符串，格式: vx.x.x
 *
 * @return sender版本字符串
 *
 */
SENDER_API const char *sender_get_version_str(void);

/**
 * @brief 清除所有未发送的异步消息
 *
 * @return int 0=成功，非0=失败
 */
SENDER_API int sender_clean_all_unsend_qbuf(void);

/**
 * @brief sender字符串形式命令接口，主要用于调试
 * @param cmd 命令字符串，如"find_all_devices";用命令"help"可查看所有命令说明；
 * @param arg_ptr 指针参数, 指向用于存放命令执行后输出内容(字符串)的buffer(由用户传入)
 * @param arg_u32 正整数参数, arg_ptr指向buffer的大小
 * 
 * @return int 非负数=成功，负数=失败
 */
SENDER_API int sender_cmd(char *cmd, void *arg_ptr, uint32_t arg_u32, ...);

#ifdef __cplusplus
}
#endif
#endif /* __ADAPS_SENDER_H__ */