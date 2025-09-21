#ifndef MISC_DEVICE_H
#define MISC_DEVICE_H

#if defined(RUN_ON_EMBEDDED_LINUX)

#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/media.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <semaphore.h>
//#include <QDebug>
#include <QDateTime>

#include "common.h"

#define DEV_NODE_LEN                            32

#define HEX_NUMBER                              16
#define DEC_NUMBER                              10
#define ADS6401_I2C_ADDR_IN_SCRIPT              0x5E
#define ADS6401_I2C_ADDR2_IN_SCRIPT             0x4A
#define VCSEL_OPN7020_I2C_ADDR_IN_SCRIPT        0x31

#define MCUCTRL_I2C_ADDR_4_FLOOD_IN_SCRIPT      0x60

#define SCRIPT_LINE_LENGTH                      256
#define ADS6401_REG_ADDR_4_WORK_START           0xAB

typedef enum {
    I2C_Write,
    I2C_Read,
    MIPI_Read,
    Swift_Delay,
    Comment,
    Block_Write,
    Block_Read,
    Slave_Addr
} ScriptType;

typedef struct {
    ScriptType     type;
    int            i2c_addr; // or sleep time in ms for delay command
    int            reg_addr;
    int            reg_val;
    char           note[200];
} ScriptItem;



class Misc_Device
{

public:
    Misc_Device();
    ~Misc_Device();

    int read_dtof_runtime_status_param(float *temperature);
    void* get_dtof_calib_eeprom_param(void);
    void* get_dtof_exposure_param(void);
    void* get_dtof_runtime_status_param(void);
    int write_device_register(register_op_data_t *reg);
    int read_device_register(register_op_data_t *reg);
    int read_dtof_exposure_param(void);
    int write_dtof_initial_param(struct adaps_dtof_intial_param *param);
    int get_dtof_module_static_data(void **pp_module_static_data, void **pp_eeprom_data_buffer, uint32_t *eeprom_data_size);
    int send_down_loaded_roisram_data_size(const uint32_t roi_sram_size);
    int send_down_external_config(const UINT8 workMode, const uint32_t script_buf_size, const uint8_t* script_buf);
    int update_eeprom_data(UINT8 *buf, UINT32 offset, UINT32 length);

private:

    void* mmap_buffer_base;
    u32 mmap_buffer_max_size;
    sem_t misc_semLock;

    char        devnode_4_misc[DEV_NODE_LEN];
    int         fd_4_misc;
    swift_spot_module_eeprom_data_t *p_spot_module_eeprom;
    swift_flood_module_eeprom_data_t *p_flood_module_eeprom;
    struct adaps_dtof_module_static_data module_static_data;
    struct adaps_dtof_runtime_status_param last_runtime_status_param;
    struct adaps_dtof_exposure_param exposureParam;
    u8* mapped_eeprom_data_buffer;
    u8* mapped_script_sensor_settings;
    uint16_t sensor_reg_setting_cnt;
    uint16_t vcsel_reg_setting_cnt;
    u8* mapped_script_vcsel_settings;           // opn7020 for spot module, PhotonIC5015 for flood module
    u8* mapped_roi_sram_data;

    int read_dtof_module_static_data(void);
    int check_crc32_4_flood_calib_eeprom_param(void);
    bool save_dtof_calib_eeprom_param(void *buf, int len);
    bool check_crc8_4_eeprom_item(uint8_t *pEEPROMData, uint32_t offset, uint32_t length, uint8_t savedCRC, const char *tag);
    int check_crc8_4_spot_calib_eeprom_param(void);
    int get_next_line(const uint8_t *buffer, size_t buffer_len, size_t *pos, char *line, size_t line_len);
    int LoadItemsFromBuffer(const uint32_t ulBufferLen, const uint8_t* pucBuffer, ScriptItem* items, uint32_t* number);
    int parse_items(const uint32_t ulItemsCount, const ScriptItem *pstrItems);
    int write_external_config_script(external_config_script_param_t *param);
    int write_external_roisram_data_size(external_roisram_data_size_t *param);

};

#endif // RUN_ON_EMBEDDED_LINUX

#endif // MISC_DEVICE_H


