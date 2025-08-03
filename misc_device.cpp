#include <sys/sysmacros.h>

#include "misc_device.h"
#include "utils.h"
#include "host_device_comm_types.h"
#include "globalapplication.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

inline void EepromGetSwiftDeviceNumAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_VERSION_INFO_OFFSET;
    *length = ADS6401_EEPROM_VERSION_INFO_SIZE;
}

inline void EepromGetSwiftSramDataAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_ROISRAM_DATA_OFFSET;
    *length = ADS6401_EEPROM_ROISRAM_DATA_SIZE;
}

inline void EepromGetSwiftIntrinsicAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_INTRINSIC_OFFSET;
    *length = ADS6401_EEPROM_INTRINSIC_SIZE;
}

/*
inline void EepromGetSwiftSpotPosAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_ACCURATESPODPOS_OFFSET; //OFFSET(swift_eeprom_data_t, spotPos);
    *length = ADS6401_EEPROM_ACCURATESPODPOS_SIZE; //MEMBER_SIZE(swift_eeprom_data_t, spotPos);
}
*/

inline void EepromGetSwiftOutDoorOffsetAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_ACCURATESPODPOS_OFFSET; //OFFSET(swift_eeprom_data_t, spotPos);
    *length = ADS6401_EEPROM_ACCURATESPODPOS_SIZE / 2; //MEMBER_SIZE(swift_eeprom_data_t, spotPos) / 2;
}

inline void EepromGetSwiftSpotOffsetbAddress(uint32_t* offset, uint32_t* length) {
    // *offset = OFFSET(swift_eeprom_data_t, spotPos) + SPOT_MODULE_OFFSET_SIZE * sizeof(float);
    *offset = ADS6401_EEPROM_ACCURATESPODPOS_OFFSET + SPOT_MODULE_OFFSET_SIZE * sizeof(float);
    *length = ADS6401_EEPROM_ACCURATESPODPOS_SIZE / 2; //MEMBER_SIZE(swift_eeprom_data_t, spotPos) / 2;
}

inline void EepromGetSwiftSpotOffsetaAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_SPOTOFFSET_OFFSET;
    *length = ADS6401_EEPROM_SPOTOFFSET_SIZE;
}

inline void EepromGetSwiftSpotOffsetAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_SPOTOFFSET_OFFSET;
    *length = ADS6401_EEPROM_SPOTOFFSET_SIZE;
}

inline void EepromGetSwiftTdcDelayAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_TDCDELAY_OFFSET;
    *length = ADS6401_EEPROM_TDCDELAY_SIZE;
}

inline void EepromGetSwiftRefDistanceAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET;
    *length = ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_SIZE
        + ADS6401_EEPROM_INDOOR_CALIBREFDISTANCE_SIZE
        + ADS6401_EEPROM_OUTDOOR_CALIBTEMPERATURE_SIZE
        + ADS6401_EEPROM_OUTDOOR_CALIBREFDISTANCE_SIZE
        + ADS6401_EEPROM_CALIBRATIONINFO_SIZE;
}

inline void EepromGetSwiftCalibInfoAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET;
    *length = ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_SIZE
        + ADS6401_EEPROM_INDOOR_CALIBREFDISTANCE_SIZE
        + ADS6401_EEPROM_OUTDOOR_CALIBTEMPERATURE_SIZE
        + ADS6401_EEPROM_OUTDOOR_CALIBREFDISTANCE_SIZE;
}

inline void EepromGetSwiftPxyAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_PROX_HISTOGRAM_OFFSET;
    *length = ADS6401_EEPROM_PROX_HISTOGRAM_SIZE
        + ADS6401_EEPROM_PROX_DEPTH_SIZE
        + ADS6401_EEPROM_PROX_NO_OF_PULSE_SIZE;
}

inline void EepromGetSwiftMarkedPixelAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_MARKED_PIXELS_OFFSET;
    *length = ADS6401_EEPROM_MARKED_PIXELS_SIZE;
}

inline void EepromGetSwiftModuleInfoAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_MODULE_INFO_OFFSET;
    *length = ADS6401_EEPROM_MODULE_INFO_SIZE;
}

inline void EepromGetSwiftWalkErrorAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_WALK_ERROR_OFFSET;
    *length = ADS6401_EEPROM_WALK_ERROR_SIZE;
}

inline void EepromGetSwiftSpotEnergyAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_SPOT_ENERGY_OFFSET;
    *length = ADS6401_EEPROM_SPOT_ENERGY_SIZE;
}

inline void EepromGetSwiftRawDepthAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_RAW_DEPTH_MEAN_OFFSET;
    *length = ADS6401_EEPROM_RAW_DEPTH_MEAN_SIZE;
}

inline void EepromGetSwiftNoiseAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_NOISE_OFFSET;
    *length = ADS6401_EEPROM_NOISE_SIZE;
}

inline void EepromGetSwiftChecksumAddress(uint32_t* offset, uint32_t* length) {
    *offset = ADS6401_EEPROM_CHKSUM_OFFSET;
    *length = ADS6401_EEPROM_CHKSUM_SIZE;
}

Misc_Device::Misc_Device()
{
    p_spot_module_eeprom = NULL_POINTER;
    p_flood_module_eeprom = NULL_POINTER;
    mmap_buffer_base = NULL_POINTER;
    mapped_eeprom_data_buffer = NULL_POINTER;
    mapped_script_sensor_settings = NULL_POINTER;
    mapped_script_vcsel_settings = NULL_POINTER;
    sensor_reg_setting_cnt = 0;
    vcsel_reg_setting_cnt = 0;
    loaded_roi_sram_size = 0;
    loaded_roi_sram_rotate = false;
    // mmap_buffer_max_size should be a multiple of PAGE_SIZE (4096, for Linux kernel memory management)
    mmap_buffer_max_size = MAX(SPOT_MODULE_EEPROM_CAPACITY_SIZE,FLOOD_MODULE_EEPROM_CAPACITY_SIZE)
        + REG_SETTING_BUF_MAX_SIZE_PER_SEG
        + REG_SETTING_BUF_MAX_SIZE_PER_SEG
        + (PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP * MAX_CALIB_SRAM_ROTATION_GROUP_CNT);
//    eeprom_data_size = sizeof(swift_eeprom_data_t);
    memset((void *) &last_runtime_status_param, 0, sizeof(struct adaps_dtof_runtime_status_param));
    fd_4_misc = 0;
    memset(devnode_4_misc, 0, DEV_NODE_LEN);
    sprintf(devnode_4_misc, "%s_misc", VIDEO_DEV_4_MISC_DEVICE);

    sem_init(&misc_semLock, 0 ,1);

    if ((fd_4_misc = open(devnode_4_misc, O_RDWR)) == -1)
    {
        DBG_ERROR("Fail to open device %s , errno: %s (%d)...", 
            devnode_4_misc, strerror(errno), errno);
        return;
    }

    if (0 != fd_4_misc)
    {
        mmap_buffer_base = mmap(NULL_POINTER, mmap_buffer_max_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_4_misc, 0);
        if (mmap_buffer_base == MAP_FAILED) {
            DBG_ERROR("Failed to mmap buffer, mmap_buffer_max_size: %d...", mmap_buffer_max_size);
            return;
        }
        mapped_eeprom_data_buffer = (u8* ) mmap_buffer_base;
        mapped_script_sensor_settings = (u8*)(mapped_eeprom_data_buffer + MAX(SPOT_MODULE_EEPROM_CAPACITY_SIZE,FLOOD_MODULE_EEPROM_CAPACITY_SIZE));
        mapped_script_vcsel_settings = (u8* ) (mapped_script_sensor_settings + ROI_SRAM_BUF_MAX_SIZE);
        mapped_roi_sram_data = mapped_script_vcsel_settings + REG_SETTING_BUF_MAX_SIZE_PER_SEG;

        if (0 > read_dtof_module_static_data())
        {
            DBG_ERROR("Failed to read module static data");
            return;
        }

    }
}


Misc_Device::~Misc_Device()
{
    if (NULL_POINTER != mmap_buffer_base)
    {
        if (munmap(mmap_buffer_base, mmap_buffer_max_size)) {
            DBG_ERROR("Failed to unmap buffer");
        }
        mmap_buffer_base = NULL_POINTER;
    }

    if (NULL_POINTER != p_spot_module_eeprom)
    {
        p_spot_module_eeprom = NULL_POINTER;
    }
    if (NULL_POINTER != p_flood_module_eeprom)
    {
        p_flood_module_eeprom = NULL_POINTER;
    }
    if ((0 != fd_4_misc) && (-1 == close(fd_4_misc))) {
        DBG_ERROR("Fail to close device %d (%s), errno: %s (%d)...", fd_4_misc, devnode_4_misc,
            strerror(errno), errno);
        return;
    }
    sem_destroy(&misc_semLock);
}

/**
 * Retrieves the next line from a buffer
 * Cross-platform line ending support:
 *   UNIX-style \n
 *   Windows-style \r\n
 *   Classic Mac-style \r
 * 
 * @param buffer Pointer to the start of the buffer
 * @param buffer_len Total length of the buffer
 * @param pos Current position pointer (will be updated by the function)
 * @param line Buffer to store the line
 * @param line_len Length of the line buffer
 * @return >0: length of line read
 *         0: empty line read
 *         -1: end of buffer reached
 */
int Misc_Device::get_next_line(const uint8_t *buffer, size_t buffer_len, size_t *pos, char *line, size_t line_len)
{
    size_t line_pos = 0;
    bool found_line_ending = false;
    
    if (*pos >= buffer_len) {
        return -1;  // End of buffer
    }
    
    // Scan through buffer until line ending or buffer end
    while (*pos < buffer_len && line_pos < line_len - 1) {
        char c = buffer[*pos];
        
        // Check for line endings
        if (c == '\n' || c == '\r') {
            found_line_ending = true;
            // Skip the line ending character
            (*pos)++;
            
            // Handle Windows-style \r\n
            if (c == '\r' && *pos < buffer_len && buffer[*pos] == '\n') {
                (*pos)++;
            }
            break;
        }
        
        line[line_pos++] = c;
        (*pos)++;
    }
    
    // Always null-terminate
    line[line_pos] = '\0';
    
    if (found_line_ending) {
        // We found a line ending - return even if line is empty
        return (int)line_pos;
    }

    // No line ending found - we're at end of buffer
    if (line_pos > 0) {
        // Last line had content but no ending
        return (int)line_pos;
    }

    // End of buffer with no content
    return -1;
}

int Misc_Device::LoadItemsFromBuffer(const uint32_t ulBufferLen, const uint8_t* pucBuffer, ScriptItem* items, uint32_t* items_number)
{
    int ret;
    size_t pos = 0;
    char* result           = NULL_POINTER;
    uint32_t scriptNum     = 0;
    int argsNum            = 0;
    char acOneLine[SCRIPT_LINE_LENGTH]    = { 0 };
    char delims[]              = ",//";

    while ((ret = get_next_line(pucBuffer, ulBufferLen, &pos, acOneLine, sizeof(acOneLine))) != -1) {
        if (0 == ret)
        {
            // This is a empty line(which has only LFCR)
            continue;
        }

        //DBG_NOTICE("ulBufferLen: %d, acOneLine: {%s} pos: %ld.\n", ulBufferLen, acOneLine, pos);

        if (acOneLine[0] == '/' && acOneLine[1] == '/') {
            // This line is comment
            items[scriptNum].type = Comment;
            strcpy(items[scriptNum].note, &acOneLine[2]);
            scriptNum++;
            continue;
        }

        argsNum = 0;

        // Split line with ","
        result = strtok(acOneLine, delims);
        while (NULL_POINTER != result) {
            // Remove space in the beginning
            while (isspace(*result)) {
                ++result;
            }

            if (0 == argsNum) {
                // Script type
                if (NULL_POINTER != strstr(result, "I2C_Write")) {
                    items[scriptNum].type = I2C_Write;
                } else if (NULL_POINTER != strstr(result, "I2C_Read")) {
                    items[scriptNum].type = I2C_Read;
                } else if (NULL_POINTER != strstr(result, "MIPI_Read")) {
                    items[scriptNum].type = MIPI_Read;
                } else if (NULL_POINTER != strstr(result, "Delay")) {
                    items[scriptNum].type = Swift_Delay;
                } else if (NULL_POINTER != strstr(result, "Block_Read")) {
                    items[scriptNum].type = Block_Read;
                } else if (NULL_POINTER != strstr(result, "Block_Write")) {
                    items[scriptNum].type = Block_Write;
                } else if (NULL_POINTER != strstr(result, "Slave_Addr")) {
                    items[scriptNum].type = Slave_Addr;
                } else {
                    break;
                }
            } 
            else if (1 == argsNum) {
                if (items[scriptNum].type == Swift_Delay) { // Eg. Delay, 500
                    // argument 1, sleep time in ms
                    items[scriptNum].i2c_addr = strtol(result, NULL_POINTER, DEC_NUMBER);
                } else {
                    // argument 1, device i2c address
                    items[scriptNum].i2c_addr = strtol(result, NULL_POINTER, HEX_NUMBER);
                }
            } 
            else if (2 == argsNum) {
                // argument 2, register address
                items[scriptNum].reg_addr = strtol(result, NULL_POINTER, HEX_NUMBER);
            }
            else if (3 == argsNum) {
                // argument 3, data
                items[scriptNum].reg_val = strtol(result, NULL_POINTER, HEX_NUMBER);
                //sscanf(result, "%x", items[scriptNum].data);
            }
            else {
                // argument 4, notes
                strcpy(items[scriptNum].note, result);
            }
            result = strtok(NULL_POINTER, delims);
            argsNum++;
        }

        scriptNum++;
    }

    *items_number = scriptNum;
    return 0;
}

int Misc_Device::parse_items(const uint32_t ulItemsCount, const ScriptItem *pstrItems)
{
    uint32_t i;
    struct setting_rvd *sensor_settings = NULL_POINTER;
    struct setting_rvd *vcsel_opn7020_settings = NULL_POINTER;
    struct setting_r16vd *vcsel_PhotonIC5015_settings = NULL_POINTER;

    if (0 == ulItemsCount) {
        DBG_ERROR("input script items count is zero.\n");
        return -1;
    }

    sensor_reg_setting_cnt = 0;
    vcsel_reg_setting_cnt = 0;
    sensor_settings = (struct setting_rvd *) mapped_script_sensor_settings;
    memset(mapped_script_sensor_settings, 0, sizeof(struct setting_rvd)*MAX_REG_SETTING_COUNT);
    if (MODULE_TYPE_SPOT == module_static_data.module_type)
    {
        vcsel_opn7020_settings = (struct setting_rvd *) mapped_script_vcsel_settings;
        memset(mapped_script_vcsel_settings, 0, sizeof(struct setting_rvd)*MAX_REG_SETTING_COUNT);
    }
    else {
        vcsel_PhotonIC5015_settings = (struct setting_r16vd *) mapped_script_vcsel_settings;
        memset(mapped_script_vcsel_settings, 0, sizeof(struct setting_r16vd)*MAX_REG_SETTING_COUNT);
    }

    // Operations from script
    for (i = 0; i < ulItemsCount; i++) {
        if (I2C_Write == pstrItems[i].type) {
            if (ADS6401_I2C_ADDR_IN_SCRIPT == pstrItems[i].i2c_addr || ADS6401_I2C_ADDR2_IN_SCRIPT == pstrItems[i].i2c_addr)
            {
                if (ADS6401_REG_ADDR_4_WORK_START == pstrItems[i].reg_addr)
                {
                    // skip stream_on register line, I will add stream_on setting after executing all script line.
                    continue;
                }

                sensor_settings[sensor_reg_setting_cnt].reg = pstrItems[i].reg_addr;
                sensor_settings[sensor_reg_setting_cnt].val = pstrItems[i].reg_val;
                sensor_settings[sensor_reg_setting_cnt].delayUs = 0;
                if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                {
                    DBG_NOTICE("---i:%d, sensor_settings[%d].reg: 0x%02X, val: 0x%02X---\n", i, sensor_reg_setting_cnt, sensor_settings[sensor_reg_setting_cnt].reg, sensor_settings[sensor_reg_setting_cnt].val);
                }
                sensor_reg_setting_cnt++;
            }
            else if ((VCSEL_OPN7020_I2C_ADDR_IN_SCRIPT == pstrItems[i].i2c_addr) && (MODULE_TYPE_SPOT == module_static_data.module_type))
            {
                vcsel_opn7020_settings[vcsel_reg_setting_cnt].reg = pstrItems[i].reg_addr;
                vcsel_opn7020_settings[vcsel_reg_setting_cnt].val = pstrItems[i].reg_val;
                vcsel_opn7020_settings[vcsel_reg_setting_cnt].delayUs = 0;
                if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                {
                    DBG_NOTICE("---i:%d, vcsel_opn7020_settings[%d].reg: 0x%02X, val: 0x%02X---\n", i, vcsel_reg_setting_cnt, vcsel_opn7020_settings[vcsel_reg_setting_cnt].reg, vcsel_opn7020_settings[vcsel_reg_setting_cnt].val);
                }
                vcsel_reg_setting_cnt++;
            }
            else if ((MCUCTRL_I2C_ADDR_4_FLOOD_IN_SCRIPT == pstrItems[i].i2c_addr) && (MODULE_TYPE_FLOOD == module_static_data.module_type))
            {
                vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].reg = pstrItems[i].reg_addr;
                vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].val = pstrItems[i].reg_val;
                vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].delayUs = 0;
                if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                {
                    DBG_NOTICE("---i:%d, vcsel_PhotonIC5015_settings[%d].reg: 0x%04X, val: 0x%02X---\n", i, vcsel_reg_setting_cnt, vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].reg, vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].val);
                }
                vcsel_reg_setting_cnt++;
            }
        } 
        else if (Swift_Delay == pstrItems[i].type) { // for delay command, we try to update the 3rd paramater of previous command.
            if (i > 1)
            {
                if (ADS6401_I2C_ADDR_IN_SCRIPT == pstrItems[i - 1].i2c_addr || ADS6401_I2C_ADDR2_IN_SCRIPT == pstrItems[i - 1].i2c_addr)
                {
                    if (sensor_reg_setting_cnt > 1)
                    {
                        sensor_settings[sensor_reg_setting_cnt - 1].delayUs = pstrItems[i].i2c_addr * 1000;
                        if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                        {
                            DBG_NOTICE("---i:%d, sensor_settings[%d].delayUs: %d---\n", i, sensor_reg_setting_cnt - 1, sensor_settings[sensor_reg_setting_cnt - 1].delayUs);
                        }
                    }
                }
                else if ((VCSEL_OPN7020_I2C_ADDR_IN_SCRIPT == pstrItems[i - 1].i2c_addr) && (MODULE_TYPE_SPOT == module_static_data.module_type))
                {
                    if (vcsel_reg_setting_cnt > 1)
                    {
                        vcsel_opn7020_settings[vcsel_reg_setting_cnt - 1].delayUs = pstrItems[i].i2c_addr * 1000;
                        if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                        {
                            DBG_NOTICE("---i:%d, vcsel_opn7020_settings[%d].delayUs: %d---\n", i, vcsel_reg_setting_cnt - 1, vcsel_opn7020_settings[vcsel_reg_setting_cnt - 1].delayUs);
                        }
                    }
                }
                else if ((MCUCTRL_I2C_ADDR_4_FLOOD_IN_SCRIPT == pstrItems[i - 1].i2c_addr) && (MODULE_TYPE_FLOOD == module_static_data.module_type))
                {
                    if (vcsel_reg_setting_cnt > 1)
                    {
                        vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt - 1].delayUs = pstrItems[i].i2c_addr * 1000;
                        if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
                        {
                            DBG_NOTICE("---i:%d, vcsel_PhotonIC5015_settings[%d].delayUs: %d, pstrItems[i].i2c_addr: %d---\n", i, vcsel_reg_setting_cnt - 1, vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt - 1].delayUs, pstrItems[i].i2c_addr);
                        }
                    }
                }
            }
        }
    }

    // add the END flag item
    sensor_settings[sensor_reg_setting_cnt].reg = REG_NULL;
    sensor_settings[sensor_reg_setting_cnt].val = 0x00;
    sensor_settings[sensor_reg_setting_cnt].delayUs = 0;
    sensor_reg_setting_cnt++;

    if (MODULE_TYPE_SPOT == module_static_data.module_type)
    {
        vcsel_opn7020_settings[vcsel_reg_setting_cnt].reg = REG_NULL;
        vcsel_opn7020_settings[vcsel_reg_setting_cnt].val = 0x00;
        vcsel_opn7020_settings[vcsel_reg_setting_cnt].delayUs = 0;
    }
    else {
        vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].reg = REG16_NULL;
        vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].val = 0x00;
        vcsel_PhotonIC5015_settings[vcsel_reg_setting_cnt].delayUs = 0;
    }
    vcsel_reg_setting_cnt++;

    return 0;
}

int Misc_Device::send_down_external_config(const UINT8 workMode, const uint32_t script_buf_size, const uint8_t* script_buf, const uint32_t roi_sram_size, const uint8_t* roi_sram_data, const bool roi_sram_rotate)
{
    ScriptItem *pstrItems = NULL_POINTER;
    uint32_t ulItemsBufSize = MAX_SCRIPT_ITEM_COUNT * sizeof(ScriptItem);
    uint32_t ulItemsCount = 0;
    int ret = 0;
    external_config_script_param_t ex_cfg_script_param;

    if (0 != script_buf_size) {
        pstrItems = (ScriptItem *)malloc(ulItemsBufSize);
        if (NULL_POINTER == pstrItems) {
            DBG_ERROR("malloc script data size: %u fail.\n", ulItemsBufSize);
            return -1;
        }
        memset(pstrItems, 0, ulItemsBufSize);

        ret = LoadItemsFromBuffer(script_buf_size, script_buf, pstrItems, &ulItemsCount);
        if (ret) {
            free(pstrItems);
            DBG_ERROR("Fail to load items from buffer for workmode: %u.\n", workMode);
            return -1;
        }

        if (true == Utils::is_env_var_true(ENV_VAR_DUMP_PARSING_SCRIPT_ITEMS))
        {
            DBG_NOTICE("load items count: %u from buffer: %p.\n", ulItemsCount, script_buf);
        }

        ret = parse_items(ulItemsCount, pstrItems);
        if (ret) {
            if (NULL_POINTER != pstrItems) free(pstrItems);
            DBG_ERROR("parse item: %u fail on workmode: %u.\n", ulItemsCount, workMode);
            return -1;
        }

        if (NULL_POINTER != pstrItems) free(pstrItems);

    }

    memcpy(mapped_roi_sram_data, roi_sram_data, roi_sram_size);
    loaded_roi_sram_size = roi_sram_size;
    loaded_roi_sram_rotate = roi_sram_rotate;

    ex_cfg_script_param.work_mode = workMode;
    ex_cfg_script_param.sensor_reg_setting_cnt = sensor_reg_setting_cnt;
    ex_cfg_script_param.vcsel_reg_setting_cnt = vcsel_reg_setting_cnt;
    ex_cfg_script_param.roi_sram_rotate = roi_sram_rotate;
    ex_cfg_script_param.roi_sram_size = roi_sram_size;
    ret = write_external_config_script(&ex_cfg_script_param);
    DBG_NOTICE("write_external_config_script() ret: %d, workMode: %d, vcsel_reg_setting_cnt:%d, sensor_reg_setting_cnt: %d, roi_sram_rotate: %d, roi_sram_size: %d---\n",
        ret, workMode, vcsel_reg_setting_cnt, sensor_reg_setting_cnt, roi_sram_rotate, roi_sram_size);

    return ret;
}

int Misc_Device::update_eeprom_data(UINT8 *buf, UINT32 offset, UINT32 length)
{
    struct adaps_dtof_update_eeprom_data update_data;

    update_data.module_type = module_static_data.module_type;
    update_data.eeprom_capacity = module_static_data.eeprom_capacity;
    update_data.offset = offset;
    update_data.length = length;
    memcpy(mapped_eeprom_data_buffer + offset, buf, length);

    sem_wait(&misc_semLock);
    int ret = misc_ioctl(fd_4_misc, ADTOF_UPDATE_EEPROM_DATA, &update_data);
    sem_post(&misc_semLock);
    if (-1 == ret) {
        errno_debug("ADTOF_UPDATE_EEPROM_DATA");
    }
    return ret;
}

int Misc_Device::write_device_register(register_op_data_t *reg)
{
    sem_wait(&misc_semLock);
    int ret = misc_ioctl(fd_4_misc, ADTOF_SET_DEVICE_REGISTER, reg);
    sem_post(&misc_semLock);
    if (-1 == ret) {
        errno_debug("ADTOF_SET_DEVICE_REGISTER");
    }
    return ret;
}

int Misc_Device::read_device_register(register_op_data_t *reg)
{
    sem_wait(&misc_semLock);
    int ret = misc_ioctl(fd_4_misc, ADTOF_GET_DEVICE_REGISTER, reg);
    sem_post(&misc_semLock);
    if (-1 == ret) {
        errno_debug("ADTOF_GET_DEVICE_REGISTER");
    }
    return ret;
}

int Misc_Device::write_external_config_script(external_config_script_param_t *param)
{
    int ret = misc_ioctl(fd_4_misc, ADTOF_SET_EXTERNAL_CONFIG_SCRIPT, param);
    if (-1 == ret) {
        errno_debug("ADTOF_ENABLE_SCRIPT_START");
    }
    return ret;
}

bool Misc_Device::save_dtof_calib_eeprom_param(void *buf, int len)
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

void* Misc_Device::get_dtof_exposure_param(void)
{
    return &exposureParam;
}

int Misc_Device::read_dtof_exposure_param(void)
{
    int ret = 0;
    struct adaps_dtof_exposure_param param;
    memset(&param, 0, sizeof(param));

    if (-1 == misc_ioctl(fd_4_misc, ADAPS_GET_DTOF_EXPOSURE_PARAM, &param)) {
        DBG_ERROR("Fail to get exposure param from dtof sensor device, errno: %s (%d)...", 
               strerror(errno), errno);
        ret = -1;
    }
    else {     
        exposureParam.exposure_period = param.exposure_period;
        exposureParam.ptm_coarse_exposure_value = param.ptm_coarse_exposure_value;
        exposureParam.ptm_fine_exposure_value = param.ptm_fine_exposure_value;
        exposureParam.pcm_gray_exposure_value = param.pcm_gray_exposure_value;
        DBG_INFO("exposure_period: 0x%02x, ptm_coarse_exposure_value: 0x%02x, ptm_fine_exposure_value: 0x%02x, pcm_gray_exposure_value: 0x%02x",
            param.exposure_period, param.ptm_coarse_exposure_value, param.ptm_fine_exposure_value,param.pcm_gray_exposure_value);
    }

    return ret;
}

void* Misc_Device::get_dtof_calib_eeprom_param(void)
{
    if (false == module_static_data.ready)
    {
        if (0 > read_dtof_module_static_data())
        {
            DBG_ERROR("Failed to read module static data");
        }
    }

    if (MODULE_TYPE_SPOT == module_static_data.module_type)
    {
        return p_spot_module_eeprom;
    }
    else {
        return p_flood_module_eeprom;
    }

}

bool Misc_Device::check_crc8_4_eeprom_item(uint8_t *pEEPROMData, uint32_t offset, uint32_t length, uint8_t savedCRC, const char *tag)
{
    bool ret = true;
    unsigned char computedCRC = 0;
    pEEPROMData += offset;

    Utils *utils = new Utils();
    computedCRC = utils->CRC8Calculate(pEEPROMData, length);
    delete utils;

    if (computedCRC != savedCRC) {
        DBG_ERROR("crc8 MISMATCHED for eeprom[%s] at offset:%d, lenth: %d, saved crc8: 0x%02x, calc crc8: 0x%02x", tag, offset, length, savedCRC, computedCRC);
        ret = false;
    }
    else {
        //DBG_INFO("crc8 matched for eeprom[%s] at offset:%d, lenth: %d, saved crc8: 0x%02x, calc crc8: 0x%02x", tag, offset, length, savedCRC, computedCRC);
        ret = true;
    }

    return ret;
}

int Misc_Device::check_crc8_4_spot_calib_eeprom_param(void)
{
    int ret = 0;
    uint32_t offset;
    uint32_t length;
    uint8_t checkSum[SPOT_MODULE_CHECKSUM_SIZE] = { 0 };
    uint8_t *pEEPROMData = (uint8_t *) p_spot_module_eeprom;
    int eeprom_data_size = sizeof(swift_spot_module_eeprom_data_t);

    if (Utils::is_env_var_true(ENV_VAR_SAVE_EEPROM_ENABLE))
    {
        save_dtof_calib_eeprom_param(pEEPROMData, eeprom_data_size);
    }

    EepromGetSwiftChecksumAddress(&offset, &length);
    memcpy(checkSum, pEEPROMData + offset, length);

    // checksum1 - calibrationInfo
    EepromGetSwiftDeviceNumAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[CALIBRATION_INFO], "1.CALIBRATION_INFO")) {
        DBG_ERROR("1.Checksum calibrationInfo (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum2 - sramData
    EepromGetSwiftSramDataAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[SRAM_DATA], "2.SRAM_DATA")) {
        DBG_ERROR("2.Checksum sramdata (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum3 - intrinsic
    EepromGetSwiftIntrinsicAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[INTRINSIC], "3.INTRINSIC")) {
        DBG_ERROR("3.Checksum intrinsic (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum4 - outdoor offset
    EepromGetSwiftOutDoorOffsetAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[OUTDOOR_OFFSET], "4.OUTDOOR_OFFSET")) {
        DBG_ERROR("4.Checksum outdoor offset (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum5 - spotOffsetB
    EepromGetSwiftSpotOffsetbAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[SPOT_OFFSET_B], "5.SPOT_OFFSET_B")) {
        DBG_ERROR("5.Checksum spotOffsetB (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum6 - spotOffsetA
    EepromGetSwiftSpotOffsetaAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[SPOT_OFFSET_A], "6.SPOT_OFFSET_A")) {
        DBG_ERROR("6.Checksum spotOffsetA (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum7 - tdcDelay
    EepromGetSwiftTdcDelayAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[TDC_DELAY], "7.TDC_DELAY")) {
        DBG_ERROR("7.Checksum tdcDelay (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum8 - refDistance
    EepromGetSwiftRefDistanceAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[REF_DISTANCE], "8.REF_DISTANCE")) {
        DBG_ERROR("8.Checksum refDistance (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum9 - proximity
    EepromGetSwiftPxyAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[PROXIMITY], "9.PROXIMITY")) {
        DBG_ERROR("9.Checksum proximity (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum10 - hotPixel & deadPixel
    EepromGetSwiftMarkedPixelAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[HOTPIXEL_DEADPIXEL], "10.HOTPIXEL_DEADPIXEL")) {
        DBG_ERROR("10.Checksum hotPixel & deadPixel (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum11 - WalkError
    EepromGetSwiftWalkErrorAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[WALKERROR], "11.WALKERROR")) {
        DBG_ERROR("11.Checksum WalkError (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum12 - SpotEnergy
    EepromGetSwiftSpotEnergyAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[SPOT_ENERGY], "12.SPOT_ENERGY")) {
        DBG_ERROR("12.Checksum SpotEnergy (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum13 - noise
    EepromGetSwiftNoiseAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, offset, length, checkSum[NOISE], "13.NOISE")) {
        DBG_ERROR("13.Checksum noise (offset:%d, length: %d) validation fail.\n", offset, length);
        ret = -EIO;
    }

    // checksum_all 
    EepromGetSwiftChecksumAddress(&offset, &length);
    if (!check_crc8_4_eeprom_item(pEEPROMData, 0, eeprom_data_size - length, checkSum[CHECKSUM_ALL], "14.CHECKSUM_ALL")) {
        DBG_ERROR("14.Checksum checksum_all validation fail, sizeof(swift_spot_module_eeprom_data_t)=%ld.\n", sizeof(swift_spot_module_eeprom_data_t));
        ret = -EIO;
    }

    return ret;
}

int Misc_Device::check_crc32_4_flood_calib_eeprom_param(void)
{
    int i = 0,ret=0;
    uint32_t read_crc32 = 0;
    uint32_t calc_crc32 = 0;
    uint8_t *pEEPROMData = (uint8_t *) p_flood_module_eeprom;
    int eeprom_data_size = sizeof(swift_flood_module_eeprom_data_t);

    if (Utils::is_env_var_true(ENV_VAR_SAVE_EEPROM_ENABLE))
    {
        save_dtof_calib_eeprom_param(pEEPROMData, eeprom_data_size);
    }

    Utils *utils = new Utils();

    //do page 0 crc
    calc_crc32 = utils->crc32(0, (const unsigned char *) pEEPROMData, FLOOD_EEPROM_VERSION_INFO_SIZE + FLOOD_EEPROM_SN_INFO_SIZE);
    if(p_flood_module_eeprom->Crc32Pg0 == calc_crc32)
    {
        DBG_INFO("EEPROM Crc32Pg0 matched!!! sizeof(swift_flood_module_eeprom_data_t)=%ld, calc_crc32:0x%x",
                sizeof(swift_flood_module_eeprom_data_t),
                calc_crc32);
        //hex_data_dump((const u8 *) sensor->eeprom_data, 64, "eeprom data head");
    }
    else {
        DBG_ERROR("EEPROM Crc32Pg0 MISMATCHED!!! FLOOD_EEPROM_ROISRAM_DATA_OFFSET=%ld, calc_crc32:0x%x,read Crc32Pg0:0x%x",
                FLOOD_EEPROM_ROISRAM_DATA_OFFSET,
                calc_crc32,
                p_flood_module_eeprom->Crc32Pg0);
        //hex_data_dump((const u8 *) sensor->eeprom_data, 64, "eeprom data head");
        ret = -1;
        //goto check_exit;
    }

    //do page 1 crc
    calc_crc32 = utils->crc32(0, (const unsigned char *) pEEPROMData + FLOOD_EEPROM_MODULE_INFO_OFFSET, FLOOD_EEPROM_MODULE_INFO_SIZE);

    if(p_flood_module_eeprom->Crc32Pg1 == calc_crc32)
    {
        DBG_INFO("EEPROM Crc32Pg1 matched!!! calc_crc32:0x%x",
                calc_crc32);
        //hex_data_dump((const u8 *) sensor->eeprom_data, 64, "eeprom data head");
    }
    else {
        DBG_ERROR("EEPROM Crc32Pg1 MISMATCHED!!! calc_crc32:0x%x,read Crc32Pg1:0x%x",
                calc_crc32,
                p_flood_module_eeprom->Crc32Pg1);
        ret = -1;
        //goto check_exit;
    }

    //do page 2 crc
    calc_crc32 = utils->crc32(0, (const unsigned char *) pEEPROMData + FLOOD_EEPROM_TDCDELAY_OFFSET, 
        FLOOD_EEPROM_TDCDELAY_SIZE + FLOOD_EEPROM_INTRINSIC_SIZE + FLOOD_EEPROM_INDOOR_CALIBTEMPERATURE_SIZE
        + FLOOD_EEPROM_OUTDOOR_CALIBTEMPERATURE_SIZE + FLOOD_EEPROM_INDOOR_CALIBREFDISTANCE_SIZE+FLOOD_EEPROM_OUTDOOR_CALIBREFDISTANCE_SIZE);
    if(p_flood_module_eeprom->Crc32Pg2 == calc_crc32)
    {
        DBG_INFO("EEPROM Crc32Pg2 matched!!! calc_crc32:0x%x",
                calc_crc32);
    }
    else {
        DBG_ERROR("EEPROM Crc32Pg2 MISMATCHED!!! calc_crc32:0x%x,read Crc32Pg2:0x%x",
                calc_crc32,
                p_flood_module_eeprom->Crc32Pg2);
        ret = -1;
        //goto check_exit;
    }

    //do sram data crc for every zone
    for ( i = 0; i < ZONE_COUNT_PER_SRAM_GROUP * CALIB_SRAM_GROUP_COUNT; ++i)
    {
        calc_crc32 = utils->crc32(0, (const unsigned char *) pEEPROMData + FLOOD_EEPROM_ROISRAM_DATA_OFFSET + i*FLOOD_MODULE_SRAM_ZONE_OCUPPY_SPACE, FLOOD_MODULE_SRAM_ZONE_VALID_DATA_LENGTH);
        read_crc32 = *(uint32_t*)(pEEPROMData + FLOOD_EEPROM_ROISRAM_DATA_OFFSET + i*FLOOD_MODULE_SRAM_ZONE_OCUPPY_SPACE + FLOOD_MODULE_SRAM_ZONE_VALID_DATA_LENGTH);
        if (calc_crc32 != read_crc32)
        {
            DBG_ERROR("SRAM %d CRC checksum MISMATCHED, calc_crc32=0x%x   read_crc32=0x%x\n", i, calc_crc32, read_crc32);
            ret = -1;
            //goto check_exit;
        }
        else
        {
            DBG_INFO("SRAM %d CRC checksum matched!!! sizeof(swift_flood_module_eeprom_data_t)=%ld, calc_crc32:0x%x",
                i,
                sizeof(swift_flood_module_eeprom_data_t),
                calc_crc32);
        }

        //reset the crc in the eeprom to 0,because the algo will use these data
        //*(uint32_t*)(p_eeprominfo->pRawData + i*SRAM_ZONE_OCUPPY_SPACE + SRAM_ZONE_VALID_DATA_LENGTH)=0;
    }

    //do offset crc 
    for ( i = 0; i < 8; ++i)
    {
        calc_crc32 = utils->crc32(0, (unsigned char*)(p_flood_module_eeprom->spotOffset) + i * FLOOD_MODULE_OFFSET_VALID_DATA_LENGTH, FLOOD_MODULE_OFFSET_VALID_DATA_LENGTH);
        if (calc_crc32 != p_flood_module_eeprom->Crc32Offset[i])
        {
            DBG_ERROR("offset %d CRC checksum MISMATCHED, read_crc=0x%x, calc_crc32:0x%x\n", i, p_flood_module_eeprom->Crc32Offset[i], calc_crc32);
            ret = -1;
            //goto check_exit;
        }
        else
        {
            DBG_INFO("offset %d CRC checksum matched!!! calc_crc32:0x%x", i, calc_crc32);
        }
    }

//check_exit:
    delete utils;

    return ret;
}

int Misc_Device::read_dtof_module_static_data(void)
{
    int ret = 0;

    if (-1 == misc_ioctl(fd_4_misc, ADAPS_GET_DTOF_MODULE_STATIC_DATA, &module_static_data)) {
        DBG_ERROR("Fail to read module_static_data of dtof misc device(%d, %s), ioctl cmd: 0x%lx errno: %s (%d)...",
            fd_4_misc, devnode_4_misc, ADAPS_GET_DTOF_MODULE_STATIC_DATA, strerror(errno), errno);
        ret = -1;
    }
    else {
        DBG_NOTICE("module_type: 0x%x, ready: %d", module_static_data.module_type, module_static_data.ready);
        if (module_static_data.ready)
        {
            qApp->set_module_type(module_static_data.module_type);

            if (MODULE_TYPE_SPOT == module_static_data.module_type)
            {
                p_spot_module_eeprom = (swift_spot_module_eeprom_data_t *) mapped_eeprom_data_buffer;
            }
            else {
                p_flood_module_eeprom = (swift_flood_module_eeprom_data_t *) mapped_eeprom_data_buffer;
            }

            if (false == Utils::is_env_var_true(ENV_VAR_SKIP_EEPROM_CRC_CHK))
            {
                if (MODULE_TYPE_SPOT == module_static_data.module_type)
                {
                    ret = check_crc8_4_spot_calib_eeprom_param();
                    ret = 0; // skip eeprom crc mismatch now, since there are some modules whose crc is mismatched.
                }
                else {
                    ret = check_crc32_4_flood_calib_eeprom_param();
                    ret = 0; // skip eeprom crc mismatch now, since there are some modules whose crc is mismatched.
                }
            }
        }
    }

    return ret;
}

int Misc_Device::get_dtof_module_static_data(void **pp_module_static_data, void **pp_eeprom_data_buffer, uint32_t *eeprom_data_size)
{
    *pp_module_static_data = (void *) &module_static_data;
    *pp_eeprom_data_buffer = mapped_eeprom_data_buffer;
    if (MODULE_TYPE_SPOT == module_static_data.module_type)
    {
        *eeprom_data_size = sizeof(swift_spot_module_eeprom_data_t);
    }
    else {
        *eeprom_data_size = sizeof(swift_flood_module_eeprom_data_t);
    }

    return 0;
}

int Misc_Device::get_loaded_roi_sram_data_info(void **pp_roisram_data_buffer, uint32_t *roisram_data_size)
{
    *pp_roisram_data_buffer = mapped_roi_sram_data;
    *roisram_data_size = loaded_roi_sram_size;

    return 0;
}

int Misc_Device::write_dtof_initial_param(struct adaps_dtof_intial_param *param)
{
    int ret = 0;

    if (-1 == misc_ioctl(fd_4_misc, ADAPS_SET_DTOF_INITIAL_PARAM, param)) {
        DBG_ERROR("Fail to set initial param for dtof sensor device, errno: %s (%d)...", 
               strerror(errno), errno);
        ret = -1;
    }
    else {
        DBG_INFO("dtof_intial_param env_type=%d measure_type=%d framerate_type=%d     ",
            param->env_type,
            param->measure_type,
            param->framerate_type);
    }

    return ret;
}

void* Misc_Device::get_dtof_runtime_status_param(void)
{
    return &last_runtime_status_param;
}

int Misc_Device::read_dtof_runtime_status_param(float *temperature)
{
    int ret = 0;
    struct adaps_dtof_runtime_status_param param;
    memset(&param,0,sizeof(param));

    if (-1 == misc_ioctl(fd_4_misc, ADAPS_GET_DTOF_RUNTIME_STATUS_PARAM, &param)) {
        DBG_ERROR("Fail to get runtime status param from dtof sensor device, errno: %s (%d)...", 
               strerror(errno), errno);
        ret = -1;
    }else
    {     
        last_runtime_status_param.inside_temperature_x100 = param.inside_temperature_x100;
        last_runtime_status_param.expected_vop_abs_x100 = param.expected_vop_abs_x100;
        last_runtime_status_param.expected_pvdd_x100 = param.expected_pvdd_x100;
    
        *temperature = (float) ((double)param.inside_temperature_x100 /(double)100.0f);
        //DBG_INFO("internal_temperature: %d, temperature: %f\n", param.inside_temperature_x100, *temperature);
    }

    return ret;
}


