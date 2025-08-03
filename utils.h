#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>
#include <QDir>
#include <QFile>
#include <QByteArray>
#include <iostream>

#include <linux/reboot.h>
#include <sys/reboot.h>
#include <unistd.h>  // 引入 getuid() 函数的声明


#include "common.h"

#define POLYNOMIAL 0xEDB88320
#define REPLAY_DATA_FILE_PATH                   "/tmp"
#define REPLAY_RAW_FILE_EXT_NAME               ".swift_raw"
#define REPLAY_DEPTH16_FILE_EXT_NAME               ".depth16"

enum test_pattern
{
    ETP_00_TO_FF = 1,
    ETP_FF_TO_00,
    ETP_FULL_00,
    ETP_FULL_55,
    ETP_FULL_AA,
    ETP_FULL_FF,
    ETP_CNT
};
    
class Utils
{

public:
    Utils();
    ~Utils();
    uint32_t crc32(uint32_t initial, const unsigned char *buf, size_t len);
    unsigned char CRC8Calculate(const unsigned char buffer[], int len);
    QByteArray loadNextFileToBuffer();
    int loadNextFileToBuffer(char *buffer, int max_read_size);
    bool is_replay_data_exist();
    void test_pattern_generate(unsigned char *write_buf, int len, int ptn_idx);
    void hexdump(const unsigned char * buf, int buf_len, const char * title);
    bool save_binary_file(const char *filename, const void *buffer, size_t size, const char *call_func, unsigned int call_line);
    void nv12_2_rgb(unsigned char *nv12, unsigned char *rgb, int width, int height);
    void yuyv_2_rgb(unsigned char *yuyv, unsigned char *rgb, int width, int height);
    bool IsASCII(const unsigned char c);
    void GetRgb4watchPoint(const u8 rgb_buffer[], const int out_frm_width, u8 x, u8 y, u8 *r, u8 *g, u8 *b);
    void GetPidTid(const char *callfunc, const int callline);
    int MD5Check4Buffer(const unsigned char* buffer, int size, const char *expected_md5_string, const char *call_func, int call_line);
    int MD5Calculate(const unsigned char* buffer, int size, const char *call_func, unsigned int call_line);

    static int system_reboot()
    {
        // 为了能够重启系统，需要获得root权限
        if (getuid() != 0)
        {
            DBG_ERROR("This program needs root privileges to reboot the system.\n");
            return 1;
        }
    
        // 使用reboot系统调用来重启
        if (reboot(LINUX_REBOOT_CMD_RESTART) == -1)
        {
            DBG_ERROR("Fail to reboot the linux, error: %d (%s)\n", errno, strerror(errno));
            return 1;
        }
    
        return 0; // 这行代码不应该被执行，除非reboot调用失败
    }

    static bool is_env_var_true(const char *var_name)
    {
        const char *env_var_value = getenv(var_name);
    
        if (env_var_value != NULL_POINTER && strcmp(env_var_value, "true") == 0) {
            return true;
        }
        return false;
    }

    static int get_env_var_intvalue(const char *var_name)
    {
        int ret = 0;
        const char *env_var_value = getenv(var_name);
    
        if (env_var_value != NULL_POINTER) {
            ret = atoi(env_var_value);
        }
    
        return ret;
    }

    static char * get_env_var_stringvalue(const char *var_name)
    {
        return getenv(var_name);
    }

private:
    QStringList fileList;
    QString replay_file_path;
    int currentIndex;
    uint32_t m_crc32_table[256];
    void generate_crc32_table();
    void loadFiles(const QString &directoryPath, const QString &fileExtension);
    unsigned char hexCharToValue(char c);
    void hexStringToByteArray(const char* hexString, unsigned char* byteArray, int byteArrayLength);
    void byteArray2HexString(const unsigned char byteArray[], int byteArrayLength, char* outputHexString);
};

#endif // UTILS_H
