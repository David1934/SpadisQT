#include <stdio.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <zlib.h>
#include <cinttypes>
#include <openssl/md5.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "utils.h"

static const unsigned char CRC8Table[] = {
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

Utils::Utils()
{
    currentIndex = 0;
    generate_crc32_table();
    replay_file_path = REPLAY_DATA_FILE_PATH;
    if (Utils::is_env_var_true(ENV_VAR_RAW_FILE_REPLAY_ENABLE))
    {
        QString fileExtension = REPLAY_RAW_FILE_EXT_NAME;
        loadFiles(replay_file_path, fileExtension);
    }
    else if (Utils::is_env_var_true(ENV_VAR_DEPTH16_FILE_REPLAY_ENABLE))
    {
        QString fileExtension = REPLAY_DEPTH16_FILE_EXT_NAME;
        loadFiles(replay_file_path, fileExtension);
    }
}

Utils::~Utils()
{
}

// 初始化crc32表
void Utils::generate_crc32_table() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 8; j > 0; j--) {
            if (crc & 1) {
                crc = (crc >> 1) ^ POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
        m_crc32_table[i] = crc;
    }
}

// 计算数据的crc32值
uint32_t Utils::crc32(uint32_t initial, const unsigned char *buf, size_t len) {
    uint32_t c = initial ^ 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        c = m_crc32_table[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFF;
}

unsigned char Utils::CRC8Calculate(const unsigned char buffer[], int len)
{
    int i;
    unsigned char crc8 = 0x77;

    for (i = len-1; i>= 0 ; i--)
    {
        crc8 = CRC8Table[(crc8 ^ buffer[i]) & 0xFF];
    }
    return crc8;
}

bool Utils::buffer_is_fully_same(const unsigned char *buffer, int len, unsigned char val)
{
    int i;

    for (i = 0; i < len ; i++)
    {
        if (buffer[i] != val)
            return false;
    }

    return true;
}

bool Utils::save_binary_file(const char *filename, const void *buffer, size_t size, const char *call_func, unsigned int call_line)
{
    FILE *fp = fopen(filename, "wb");

    if (fp == NULL_POINTER) {
        DBG_ERROR("Fail to create file %s , errno: %s (%d) call from <%s> Line: %d...", 
            filename, strerror(errno), errno, call_func, call_line);
        return false;
    }

    fwrite(buffer, size, 1, fp);
    fclose(fp);

    return true;
}

void Utils::save_depth_txt_file(void *frm_buf,unsigned int frm_sequence,int frm_len, int w, int h)
{
    const int DEPTH_MASK = 0x3FFF; // For depth16 format, the low 14 bits is distance
    QDateTime       localTime = QDateTime::currentDateTime();
    QString         currentTime = localTime.toString("yyyyMMddhhmmss");
    char *          LocalTimeStr = (char *) currentTime.toStdString().c_str();

    char path[50]={0};
    int16_t *p_temp=(int16_t*)frm_buf;

    sprintf(path,"%sframe%03d_%s_%d_depth%s",DATA_SAVE_PATH,frm_sequence, LocalTimeStr, frm_len, ".txt");
    FILE*fp = NULL_POINTER;
    fp=fopen(path, "w+");
    if (fp == NULL_POINTER)
    {
        DBG_ERROR("fopen output file %s failed!\n",  path);
        return;
    }
    for (int i = 0; i < h; i++)
    {
        int offset = i * w;
        for (int j = 0; j < w; j++)
        {
            fprintf(fp, "%6u ", (*(p_temp + offset + j)) & DEPTH_MASK);  //do not printf high confidence bits
        }
        fprintf(fp, "\n");
    }
    fflush(fp);
    fclose(fp);
    DBG_INFO("Save depth file %s success!\n",  path);

}

bool Utils::save_dtof_eeprom_calib_data_2_file(void *buf, int len)
{
    QDateTime       localTime = QDateTime::currentDateTime();
    QString         currentTime = localTime.toString("yyyyMMddhhmmss");
    char *          LocalTimeStr = (char *) currentTime.toStdString().c_str();
    char *          filename = new char[50];

    sprintf(filename, "%s%s.eeprom",DATA_SAVE_PATH,LocalTimeStr);
    Utils *utils = new Utils();
    utils->save_binary_file(filename, buf, 
        len,
        __FUNCTION__,
        __LINE__
        );
    delete utils;
    delete[] filename;

    return true;
}

// 加载指定目录下特定后缀名的文件到文件列表
void Utils::loadFiles(const QString &directoryPath, const QString &fileExtension) {
    QDir dir(directoryPath);
    QStringList filters;
    filters << "*" + fileExtension;
    fileList = dir.entryList(filters, QDir::Files);
}

// 从文件列表中获取下一个文件内容加载到buffer，循环加载
QByteArray Utils::loadNextFileToBuffer() {
    if (fileList.isEmpty()) {
        DBG_ERROR("Filelist is NULL!");
        return QByteArray();
    }

    QString currentFileName = fileList[currentIndex];
    QString fullFilePath = replay_file_path + "/" + currentFileName;
    DBG_NOTICE("load file %s, currentIndex: %d", fullFilePath.toUtf8().data(), currentIndex);
    QFile file(fullFilePath);
    if (file.open(QIODevice::ReadOnly)) {
        QByteArray buffer = file.readAll();
        file.close();

        // 更新索引，实现循环
        currentIndex = (currentIndex + 1) % fileList.size();
        return buffer;
    }

    return QByteArray();
}

int Utils::loadNextFileToBuffer(char *buffer, int max_read_size) {
    if (fileList.empty()) {
        DBG_ERROR("Filelist is NULL!");
        return -1; // 文件列表为空
    }

    QString currentFileName = fileList[currentIndex];
    QString fullFilePath = replay_file_path + "/" + currentFileName;
    
    // 打开文件（二进制模式）
    FILE *file = fopen(fullFilePath.toUtf8().data(), "rb");
    if (file == NULL) {
        return -1; // 文件打开失败
    }

    // 读取文件内容到缓冲区
    size_t bytesRead = fread(buffer, 1, max_read_size, file);
    
    // 关闭文件
    fclose(file);
    
    DBG_NOTICE("load file %s, currentIndex: %d", fullFilePath.toUtf8().data(), currentIndex);
    // 移动到下一个文件索引
    currentIndex = (currentIndex + 1) % fileList.size();
    
    // 返回实际读取的字节数
    return static_cast<int>(bytesRead);
}


bool Utils::is_replay_data_exist()
{
    bool ret;

    if (fileList.isEmpty())
    {
        ret = false;
    }
    else
    {
        ret = true;
    }

    return ret;
}

void Utils::test_pattern_generate(unsigned char *write_buf, int len, int ptn_idx)
{
    int i;

    switch (ptn_idx)
    {
        case ETP_FULL_00:
            memset(write_buf,0,len);
            break;

        case ETP_FULL_55:
            memset(write_buf,0x55,len);
            break;

        case ETP_FULL_AA:
            memset(write_buf,0xAA,len);
            break;

        case ETP_FULL_FF:
            memset(write_buf,0xFF,len);
            break;

        case ETP_00_TO_FF:
            for (i=0; i<len; i++)
            {
                write_buf[i] = (unsigned char) (i & 0xFF);
            }
            break;

        case ETP_FF_TO_00:
            for (i=0; i<len; i++)
            {
                write_buf[i] = (unsigned char) (0xFF - (i & 0xFF));
            }
            break;

        default:
            break;
    }
}

bool Utils::IsASCII(const unsigned char c)
{
    bool ret = false;

    if (c >= 32 && c < 128)
        ret = true;

    return ret;
}

void Utils::hexdump(const unsigned char * buf, int buf_len, const char * title)
{
    int             i, j, k;
    const int       NoPerLine = 16;
    char            line_buf[100];
    int             line_len;

    if (NULL_POINTER == buf) {
        DBG_ERROR("Null pointer");
        return;
    }

    if (NULL_POINTER != title) {
        DBG_PRINTK("-------%s--------\n", title);
    }

    for (i = 0; i <= buf_len / NoPerLine; i++) {
        memset(line_buf, 0, sizeof(line_buf));
        line_len = 0;

        for (j = 0; j < NoPerLine; j++) {
            k = i * NoPerLine + j;

            if (k >= buf_len)
                break;

            line_len += sprintf(line_buf + line_len, "%02x,", buf[k]);
        }

        line_len += sprintf(line_buf + line_len, "%s", "    ");

        for (j = 0; j < NoPerLine; j++) {
            k = i * NoPerLine + j;

            if (k >= buf_len)
                break;

            if (IsASCII(buf[k]))
            {
                line_len += sprintf(line_buf + line_len, "%c", buf[k]);
            }
            else {
                line_len += sprintf(line_buf + line_len, "%c", '.');
            }
        }

        DBG_PRINTK("[%4d] %s\n", i * NoPerLine, line_buf);
    }
}

void Utils::nv12_2_rgb(unsigned char *nv12 , unsigned char *rgb, int width , int height)
{
    const int nv_start = width * height;
    int  i, j, index = 0, rgb_index = 0;
    unsigned char y, u, v;
    int r, g, b, nv_index = 0;

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++){
            nv_index = i / 2 * width + j - j % 2;

            y = nv12[rgb_index];
            u = nv12[nv_start + nv_index];
            v = nv12[nv_start + nv_index + 1];

            r = y + (140 * (v - 128)) / 100;
            g = y - (34 * (u - 128)) / 100 - (71 * (v - 128)) / 100;
            b = y + (177 * (u - 128)) / 100;

            if (r > 255)   r = 255;
            if (g > 255)   g = 255;
            if (b > 255)   b = 255;
            if (r < 0)     r = 0;
            if (g < 0)     g = 0;
            if (b < 0)     b = 0;

            index = rgb_index ;
            rgb[index * 3 + 0] = r;
            rgb[index * 3 + 1] = g;
            rgb[index * 3 + 2] = b;
            rgb_index++;
        }
    }
    return;
}


void Utils::yuyv_2_rgb(unsigned char *yuyv, unsigned char *rgb, int width, int height) {
    int i, j;
    int y0, u, y1, v;
    int r, g, b;

    for (i = 0, j = 0; i < (width * height) * 2; i+=4, j+=6) {
        y0 = yuyv[i];
        u = yuyv[i + 1] - 128;
        y1 = yuyv[i + 2];
        v = yuyv[i + 3] - 128;

        r = y0 + 1.370705 * v;
        g = y0 - 0.698001 * v - 0.337633 * u;
        b = y0 + 1.732446 * u;

        /* Ensure RGB values are within range */
        rgb[j] = (r > 255) ? 255 : ((r < 0) ? 0 : r);
        rgb[j + 1] = (g > 255) ? 255 : ((g < 0) ? 0 : g);
        rgb[j + 2] = (b > 255) ? 255 : ((b < 0) ? 0 : b);

        r = y1 + 1.370705 * v;
        g = y1 - 0.698001 * v - 0.337633 * u;
        b = y1 + 1.732446 * u;

        rgb[j + 3] = (r > 255) ? 255 : ((r < 0) ? 0 : r);
        rgb[j + 4] = (g > 255) ? 255 : ((g < 0) ? 0 : g);
        rgb[j + 5] = (b > 255) ? 255 : ((b < 0) ? 0 : b);
    }
}

void Utils::GetRgb4watchPoint(const u8 rgb_buffer[], const int out_frm_width, u8 x, u8 y, u8 *r, u8 *g, u8 *b)
{
    int rawImgIdx;
    rawImgIdx = y * out_frm_width + x;

    *r = rgb_buffer[rawImgIdx * 3 + 0];
    *g = rgb_buffer[rawImgIdx * 3 + 1];
    *b = rgb_buffer[rawImgIdx * 3 + 2];
}

void Utils::GetPidTid(const char *callfunc, const int callline)
{
    pid_t pid = getpid();
    pid_t tid = syscall(SYS_gettid);

#if 0
    // 获取 POSIX 线程 ID
    pthread_t ptid = pthread_self();

    printf("--------PID: %d, TID (kernel): %d, pthread ID: %lu for <%s> Line: %d--------\n",
           pid, tid, (unsigned long)ptid, callfunc, callline);
#else
    printf("--------PID: %d, TID (kernel): %d for <%s> Line: %d--------\n",
           pid, tid, callfunc, callline);
#endif
}

unsigned char Utils::hexCharToValue(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    } else if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return 0; // Invalid hex char
}

void Utils::hexStringToByteArray(const char* hexString, unsigned char* byteArray, int byteArrayLength)
{
    int hexStringLength = strlen(hexString);
    for (int i = 0; i < hexStringLength && i/2 < byteArrayLength; i += 2) {
        byteArray[i/2] = hexCharToValue(hexString[i]) * 16 + hexCharToValue(hexString[i + 1]);
    }
}

void Utils::byteArray2HexString(const unsigned char byteArray[], int byteArrayLength, char* outputHexString)
{
    int j;
    int output_len = 0;

    for (j = 0; j < byteArrayLength; j++) {
        output_len += sprintf(outputHexString + output_len, "%02x", byteArray[j]);
    }
}

int Utils::MD5Check4Buffer(const unsigned char* buffer, int size, const char *expected_md5_string, const char *call_func, int call_line)
{
    int ret = 0;
    unsigned char calced_md5[MD5_DIGEST_LENGTH];
    unsigned char expected_md5[MD5_DIGEST_LENGTH];

    MD5(buffer, size, calced_md5);
    hexStringToByteArray(expected_md5_string, expected_md5, MD5_DIGEST_LENGTH);
    if (memcmp(expected_md5, calced_md5, MD5_DIGEST_LENGTH))
    {
        DBG_ERROR("===md5 mismatched call from <%s> Line: %d===", call_func, call_line);
        ret = -1;
    }
    else {
        DBG_INFO("===md5 matched call from <%s> Line: %d===", call_func, call_line);
    }

    return ret;
}

#if 0
int Utils::MD5Calculate(const unsigned char* buffer, int size, const char *call_func, unsigned int call_line)
{
    int ret = 0;
    unsigned char calced_md5[MD5_DIGEST_LENGTH];
    char calced_md5_string[MD5_DIGEST_LENGTH*2+1];

    MD5(buffer, size, calced_md5);
    byteArray2HexString(calced_md5, MD5_DIGEST_LENGTH, calced_md5_string);
    DBG_NOTICE("===md5 is <%s> call from <%s> Line: %d, MD5_DIGEST_LENGTH: %d===", calced_md5_string, call_func, call_line, MD5_DIGEST_LENGTH);

    return ret;
}

#else
int Utils::MD5Calculate(const unsigned char* buffer, int size, const char *call_func, unsigned int frm_sequence)
{
    int ret = 0;
    bool matched = false;
    unsigned char calced_md5[MD5_DIGEST_LENGTH];
    char calced_md5_string[MD5_DIGEST_LENGTH*2+1];
    char md5_4_first_3_subframe[] = "d1758809104f766aca66731345ab8a9f";
    char md5_4_forth_subframe[] = "97bd91b9e01d9f6a92bb551e27987a5f"; //518c6143ff199c498f6708619f01774f";

    MD5(buffer, size, calced_md5);
    byteArray2HexString(calced_md5, MD5_DIGEST_LENGTH, calced_md5_string);
    if (memcmp(calced_md5_string, md5_4_first_3_subframe, MD5_DIGEST_LENGTH*2))
    {
        if (memcmp(calced_md5_string, md5_4_forth_subframe, MD5_DIGEST_LENGTH*2))
        {
        }
        else {
            matched = true;
        }
    }
    else {
        matched = true;
    }

    if (matched)
    {
//        DBG_NOTICE("===md5 is <%s> call from <%s> Line: %d, matched===", calced_md5_string, call_func, call_line);
    }
    else {
        DBG_ERROR("===md5 is <%s> call from <%s> frm_sequence: %d, MISMATCHED===", calced_md5_string, call_func, frm_sequence);
        if (frm_sequence < 4)
        {
            char filename[50];
            
            sprintf(filename, "%s%s.depth16", DATA_SAVE_PATH, calced_md5_string);
            save_binary_file(filename, buffer, size, __FUNCTION__, __LINE__);
        }
    }

    return ret;
}
#endif

/**
 * @brief Check if a directory exists and is writable
 * @param dir_path Path to the directory
 * @return 0: Exists and writable; -1: Does not exist; -2: Exists but not writable; -3: Not a directory; Other negative values: System call error
 */
int Utils::check_dir_exist_and_writable(const char *dir_path)
{
    if (dir_path == NULL) {
        DBG_ERROR("Error: Directory path can't be blank\n");
        return -4;
    }

    struct stat stat_buf;
    int ret = stat(dir_path, &stat_buf);

    // Check if directory exists
    if (ret == -1) {
        DBG_ERROR("Directory <%s> does not exist", dir_path);
        return -1;  // Directory does not exist
    }

    // Verify it's a directory (not a file)
    if (!S_ISDIR(stat_buf.st_mode)) {
        DBG_ERROR("Error: <%s> is not a directory\n", dir_path);
        return -3;
    }

    // Check write permission for current process
    ret = access(dir_path, W_OK);
    if (ret == -1) {
        DBG_ERROR("Directory <%s> is not writable", dir_path);
        return -2;  // Exists but no write permission
    }

    // All checks passed
    return 0;
}

/**
 * @brief Check if a file/path exists (does not distinguish file types)
 * @param file_path Path to the file
 * @return 0: Exists; -1: Does not exist; Other negative values: System call error
 */
int Utils::check_file_exist(const char *file_path)
{
    if (file_path == NULL) {
        DBG_ERROR("Error: File path can't be blank\n");
        return -2;
    }

    // Use F_OK to only check existence (no permission check)
    int ret = access(file_path, F_OK);
    if (ret == -1) {
        DBG_ERROR("File %s does not exist", file_path);
        return -1;
    }

    return 0;
}

/**
 * @brief Enhanced check: Verify if path is a regular file and exists
 * @param file_path Path to the file
 * @return 0: Regular file and exists; -1: Does not exist; -2: Not a regular file; Other negative values: System call error
 */
int Utils::check_regular_file_exist(const char *file_path)
{
    if (file_path == NULL) {
        DBG_ERROR("Error: File path is a null pointer\n");
        return -3;
    }

    struct stat stat_buf;
    int ret = stat(file_path, &stat_buf);
    if (ret == -1) {
        DBG_ERROR("stat file failed");
        return -1;  // Does not exist
    }

    // Check if it's a regular file (exclude directories, symlinks, devices, etc.)
    if (!S_ISREG(stat_buf.st_mode)) {
        DBG_ERROR("Error: %s is not a regular file\n", file_path);
        return -2;
    }

    return 0;
}


