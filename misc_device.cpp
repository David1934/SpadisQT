#include <sys/sysmacros.h>

#include "misc_device.h"
#include "utils.h"
#include "host_device_comm_types.h"
#include "globalapplication.h"

#define REG_NULL                            0xFF
#define REG16_NULL                          0xFFFF

#define errno_debug(fmt)                    DBG_ERROR("%s error %d, %s\n", (fmt), errno, strerror(errno))

Misc_Device::Misc_Device()
{
    p_spot_module_eeprom = NULL_POINTER;
    p_flood_module_eeprom = NULL_POINTER;
    p_bigfov_module_eeprom = NULL_POINTER;
    mmap_buffer_base = NULL_POINTER;
    mapped_eeprom_data_buffer = NULL_POINTER;
    mapped_script_sensor_settings = NULL_POINTER;
    mapped_script_vcsel_settings = NULL_POINTER;
    sensor_reg_setting_cnt = 0;
    vcsel_reg_setting_cnt = 0;
    // mmap_buffer_max_size should be a multiple of PAGE_SIZE (4096, for Linux kernel memory management)
    mmap_buffer_max_size = MMAP_BUFFER_MAX_SIZE_4_WHOLE_EEPROM_DATA
        + REG_SETTING_BUF_MAX_SIZE_PER_SEG
        + REG_SETTING_BUF_MAX_SIZE_PER_SEG
        + (PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP * MAX_CALIB_SRAM_ROLLING_GROUP_CNT);
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
        mapped_script_sensor_settings = (u8*)(mapped_eeprom_data_buffer + MMAP_BUFFER_MAX_SIZE_4_WHOLE_EEPROM_DATA);
        mapped_script_vcsel_settings = (u8* ) (mapped_script_sensor_settings + REG_SETTING_BUF_MAX_SIZE_PER_SEG);
        mapped_roi_sram_data = mapped_script_vcsel_settings + REG_SETTING_BUF_MAX_SIZE_PER_SEG;
        qApp->set_mmap_address_4_loaded_roisram(mapped_roi_sram_data);

        if (0 > read_dtof_module_static_data())
        {
            DBG_ERROR("Failed to read module static data");
            return;
        }

    }
}


Misc_Device::~Misc_Device()
{
    DBG_NOTICE("------------fd_4_misc: %d---\n", fd_4_misc);

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
    if (NULL_POINTER != p_bigfov_module_eeprom)
    {
        p_bigfov_module_eeprom = NULL_POINTER;
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
    if (ADS6401_MODULE_SMALL_FLOOD != module_static_data.module_type)
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
            else if ((VCSEL_OPN7020_I2C_ADDR_IN_SCRIPT == pstrItems[i].i2c_addr) && (ADS6401_MODULE_SMALL_FLOOD != module_static_data.module_type))
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
            else if ((MCUCTRL_I2C_ADDR_4_FLOOD_IN_SCRIPT == pstrItems[i].i2c_addr) && (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type))
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
                else if ((VCSEL_OPN7020_I2C_ADDR_IN_SCRIPT == pstrItems[i - 1].i2c_addr) && (ADS6401_MODULE_SMALL_FLOOD != module_static_data.module_type))
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
                else if ((MCUCTRL_I2C_ADDR_4_FLOOD_IN_SCRIPT == pstrItems[i - 1].i2c_addr) && (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type))
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

    if (ADS6401_MODULE_SMALL_FLOOD != module_static_data.module_type)
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

int Misc_Device::send_down_external_config(const UINT8 workMode, const uint32_t script_buf_size, const uint8_t* script_buf)
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

    ex_cfg_script_param.work_mode = workMode;
    ex_cfg_script_param.sensor_reg_setting_cnt = sensor_reg_setting_cnt;
    ex_cfg_script_param.vcsel_reg_setting_cnt = vcsel_reg_setting_cnt;
    ret = write_external_config_script(&ex_cfg_script_param);
    DBG_NOTICE("write_external_config_script() ret: %d, workMode: %d, vcsel_reg_setting_cnt:%d, sensor_reg_setting_cnt: %d---\n",
        ret, workMode, vcsel_reg_setting_cnt, sensor_reg_setting_cnt);

    return ret;
}

int Misc_Device::send_down_loaded_roisram_data_size(const uint32_t roi_sram_size)
{
    int ret = 0;
    external_roisram_data_size_t ex_roisram_data_param;

    ex_roisram_data_param.roi_sram_size = roi_sram_size;
    ret = write_external_roisram_data_size(&ex_roisram_data_param);
    DBG_NOTICE("write_external_roisram_data_size() ret: %d, roi_sram_size: %d---\n", ret, roi_sram_size);

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
        errno_debug("ADTOF_SET_EXTERNAL_CONFIG_SCRIPT");
    }
    return ret;
}

int Misc_Device::write_external_roisram_data_size(external_roisram_data_size_t *param)
{
    int ret = misc_ioctl(fd_4_misc, ADTOF_SET_EXTERNAL_ROISRAM_DATA_SIZE, param);
    if (-1 == ret) {
        errno_debug("ADTOF_SET_EXTERNAL_ROISRAM_DATA_SIZE");
    }
    return ret;
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

    if (ADS6401_MODULE_SPOT == module_static_data.module_type)
    {
        return p_spot_module_eeprom;
    }
    else if (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type) {
        return p_flood_module_eeprom;
    }
    else {
        return p_bigfov_module_eeprom;
    }

}

int Misc_Device::dump_eeprom_data(u8* pEEPROM_Data)
{
    if (ADS6401_MODULE_SPOT == module_static_data.module_type)
    {
        swift_spot_module_eeprom_data_t *spot_module_eeprom = (swift_spot_module_eeprom_data_t *) pEEPROM_Data;

        DBG_NOTICE("char    deviceName[64]                  = [%s]", spot_module_eeprom->deviceName);
        DBG_NOTICE("float   intrinsic[9]                    = [%f,%f,%f,%f,%f,%f,%f,%f,%f]", 
            spot_module_eeprom->intrinsic[0],
            spot_module_eeprom->intrinsic[1],
            spot_module_eeprom->intrinsic[2],
            spot_module_eeprom->intrinsic[3],
            spot_module_eeprom->intrinsic[4],
            spot_module_eeprom->intrinsic[5],
            spot_module_eeprom->intrinsic[6],
            spot_module_eeprom->intrinsic[7],
            spot_module_eeprom->intrinsic[8]
            );
        DBG_NOTICE("float   offset                          = %f", spot_module_eeprom->offset);
        DBG_NOTICE("__u32   refSpadSelection                = 0x%08x", spot_module_eeprom->refSpadSelection);
        DBG_NOTICE("__u32   driverChannelOffset[4]          = [0x%x,0x%x,0x%x,0x%x]", 
            spot_module_eeprom->driverChannelOffset[0],
            spot_module_eeprom->driverChannelOffset[1],
            spot_module_eeprom->driverChannelOffset[2],
            spot_module_eeprom->driverChannelOffset[3]);
        DBG_NOTICE("__u32   distanceTemperatureCoefficient  = 0x%08x", spot_module_eeprom->distanceTemperatureCoefficient);
        DBG_NOTICE("__u32   tdcDelay[16]                    = [0x%x,0x%x,...]", spot_module_eeprom->tdcDelay[0], spot_module_eeprom->tdcDelay[1]);
        DBG_NOTICE("float   indoorCalibTemperature          = %f", spot_module_eeprom->indoorCalibTemperature);
        DBG_NOTICE("float   indoorCalibRefDistance          = %f", spot_module_eeprom->indoorCalibRefDistance);
        DBG_NOTICE("float   outdoorCalibTemperature         = %f", spot_module_eeprom->outdoorCalibTemperature);
        DBG_NOTICE("float   outdoorCalibRefDistance         = %f", spot_module_eeprom->outdoorCalibRefDistance);
        DBG_NOTICE("float   pxyDepth                        = %f", spot_module_eeprom->pxyDepth);
        DBG_NOTICE("float   pxyNumberOfPulse                = %f", spot_module_eeprom->pxyNumberOfPulse);
        DBG_NOTICE("char    moduleInfo[16]                  = [%s]", spot_module_eeprom->moduleInfo);
    }
    else if (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type) {
        swift_flood_module_eeprom_data_t *small_flood_module_eeprom = (swift_flood_module_eeprom_data_t *) pEEPROM_Data;

        DBG_NOTICE("char    Version[6]                      = [%s]", small_flood_module_eeprom->Version);
        DBG_NOTICE("char    SerialNumber[16]                = [%s]", small_flood_module_eeprom->SerialNumber);
        DBG_NOTICE("char    ModuleInfo[60]                  = [%s]", small_flood_module_eeprom->ModuleInfo);
        DBG_NOTICE("uint8_t tdcDelay[2]                     = [0x%x,0x%x]", small_flood_module_eeprom->tdcDelay[0], small_flood_module_eeprom->tdcDelay[1]);
        DBG_NOTICE("float   intrinsic[9]                    = [%f,%f,%f,%f,%f,%f,%f,%f,%f]", 
            small_flood_module_eeprom->intrinsic[0],
            small_flood_module_eeprom->intrinsic[1],
            small_flood_module_eeprom->intrinsic[2],
            small_flood_module_eeprom->intrinsic[3],
            small_flood_module_eeprom->intrinsic[4],
            small_flood_module_eeprom->intrinsic[5],
            small_flood_module_eeprom->intrinsic[6],
            small_flood_module_eeprom->intrinsic[7],
            small_flood_module_eeprom->intrinsic[8]
            );
        DBG_NOTICE("float   indoorCalibTemperature          = %f", small_flood_module_eeprom->indoorCalibTemperature);
        DBG_NOTICE("float   indoorCalibRefDistance          = %f", small_flood_module_eeprom->indoorCalibRefDistance);
        DBG_NOTICE("float   outdoorCalibTemperature         = %f", small_flood_module_eeprom->outdoorCalibTemperature);
        DBG_NOTICE("float   outdoorCalibRefDistance         = %f", small_flood_module_eeprom->outdoorCalibRefDistance);
    }
    else {
        swift_eeprom_v2_data_t *bigfov_module_eeprom = (swift_eeprom_v2_data_t *) pEEPROM_Data;
        
        DBG_NOTICE("//HEAD");
        DBG_NOTICE("uint32_t    magic_id                                = 0x%x", bigfov_module_eeprom->magic_id);
        DBG_NOTICE("uint32_t    data_structure_version                  = 0x%x", bigfov_module_eeprom->data_structure_version);
        DBG_NOTICE("uint8_t     calibrationInfo[32]                     = [%s]", bigfov_module_eeprom->calibrationInfo);
        DBG_NOTICE("uint8_t     LastCalibratedTime[32]                  = [%s]", bigfov_module_eeprom->LastCalibratedTime);
        DBG_NOTICE("uint8_t     serialNumber[BIG_FOV_MODULE_SN_LENGTH]  = [%s]", bigfov_module_eeprom->serialNumber);
        DBG_NOTICE("uint32_t    data_length                             = %d", bigfov_module_eeprom->data_length);
        DBG_NOTICE("uint32_t    data_crc32                              = 0x%x", bigfov_module_eeprom->data_crc32);
        DBG_NOTICE("uint32_t    compressed_data_length                  = %d", bigfov_module_eeprom->compressed_data_length);
        DBG_NOTICE("uint32_t    compressed_data_crc32                   = 0x%x\n", bigfov_module_eeprom->compressed_data_crc32);
        
        DBG_NOTICE("//BASIC_DATA");
        DBG_NOTICE("uint8_t     real_spot_zone_count                    = %d", bigfov_module_eeprom->real_spot_zone_count);
        DBG_NOTICE("uint8_t     tdcDelay[2]                             = [0x%x,0x%x]", bigfov_module_eeprom->tdcDelay[0], bigfov_module_eeprom->tdcDelay[1]);
        DBG_NOTICE("uint8_t     lensType                                = %d", bigfov_module_eeprom->lensType);
        DBG_NOTICE("float       indoorCalibTemperature                  = %f", bigfov_module_eeprom->indoorCalibTemperature);
        DBG_NOTICE("float       indoorCalibRefDistance                  = %f", bigfov_module_eeprom->indoorCalibRefDistance);
        DBG_NOTICE("float       outdoorCalibTemperature                 = %f", bigfov_module_eeprom->outdoorCalibTemperature);
        DBG_NOTICE("float       outdoorCalibRefDistance                 = %f", bigfov_module_eeprom->outdoorCalibRefDistance);
        DBG_NOTICE("float       intrinsic[9]                            = [%f,%f,%f,%f,%f,%f,%f,%f,%f]", 
            bigfov_module_eeprom->intrinsic[0],
            bigfov_module_eeprom->intrinsic[1],
            bigfov_module_eeprom->intrinsic[2],
            bigfov_module_eeprom->intrinsic[3],
            bigfov_module_eeprom->intrinsic[4],
            bigfov_module_eeprom->intrinsic[5],
            bigfov_module_eeprom->intrinsic[6],
            bigfov_module_eeprom->intrinsic[7],
            bigfov_module_eeprom->intrinsic[8]
            );
        DBG_NOTICE("float       rgb_intrinsic[8]                        = [%f,%f,%f,%f,%f,%f,%f,%f]", 
            bigfov_module_eeprom->rgb_intrinsic[0],
            bigfov_module_eeprom->rgb_intrinsic[1],
            bigfov_module_eeprom->rgb_intrinsic[2],
            bigfov_module_eeprom->rgb_intrinsic[3],
            bigfov_module_eeprom->rgb_intrinsic[4],
            bigfov_module_eeprom->rgb_intrinsic[5],
            bigfov_module_eeprom->rgb_intrinsic[6],
            bigfov_module_eeprom->rgb_intrinsic[7]
            );
        DBG_NOTICE("float       common_extrinsic[8]                     = [%f,%f,%f,%f,%f,%f,%f]", 
            bigfov_module_eeprom->common_extrinsic[0],
            bigfov_module_eeprom->common_extrinsic[1],
            bigfov_module_eeprom->common_extrinsic[2],
            bigfov_module_eeprom->common_extrinsic[3],
            bigfov_module_eeprom->common_extrinsic[4],
            bigfov_module_eeprom->common_extrinsic[5],
            bigfov_module_eeprom->common_extrinsic[6]
            );
        DBG_NOTICE("uint8_t     Reserved[20]\n");
    }

    return 0;
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
            uint8_t *pEEPROMData;
            int eeprom_data_size;

            qApp->set_module_type(module_static_data.module_type);

            if (true == Utils::is_env_var_true(ENV_VAR_DUMP_EEPROM_DATA))
            {
                dump_eeprom_data(mapped_eeprom_data_buffer);
            }

            if (ADS6401_MODULE_SPOT == module_static_data.module_type)
            {
                p_spot_module_eeprom = (swift_spot_module_eeprom_data_t *) mapped_eeprom_data_buffer;
                qApp->set_anchorOffset(0, 1); // set default anchor Offset (in case no host_comm) for spot module, may be changed from PC SpadisApp.

                pEEPROMData = (uint8_t *) p_spot_module_eeprom;
                eeprom_data_size = sizeof(swift_spot_module_eeprom_data_t);
            }
            else if (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type) {
                p_flood_module_eeprom = (swift_flood_module_eeprom_data_t *) mapped_eeprom_data_buffer;
                qApp->set_anchorOffset(0, 0); // non-spot module does not need anchor preprocess

                pEEPROMData = (uint8_t *) p_flood_module_eeprom;
                eeprom_data_size = sizeof(swift_flood_module_eeprom_data_t);
            }
            else {
                p_bigfov_module_eeprom = (swift_eeprom_v2_data_t *) mapped_eeprom_data_buffer;
                qApp->set_anchorOffset(0, 0); // non-spot module does not need anchor preprocess

                pEEPROMData = (uint8_t *) p_bigfov_module_eeprom;
                eeprom_data_size = sizeof(swift_eeprom_v2_data_t);
            }

            if (Utils::is_env_var_true(ENV_VAR_SAVE_EEPROM_ENABLE))
            {
                Utils *utils = new Utils();
                utils->save_dtof_eeprom_calib_data_2_file(pEEPROMData, eeprom_data_size);
                delete utils;
            }

        }
    }

    return ret;
}

int Misc_Device::get_dtof_module_static_data(void **pp_module_static_data, void **pp_eeprom_data_buffer, uint32_t *eeprom_data_size)
{
    *pp_module_static_data = (void *) &module_static_data;
    *pp_eeprom_data_buffer = mapped_eeprom_data_buffer;
    if (ADS6401_MODULE_SPOT == module_static_data.module_type)
    {
        *eeprom_data_size = sizeof(swift_spot_module_eeprom_data_t);
    }
    else if (ADS6401_MODULE_SMALL_FLOOD == module_static_data.module_type) {
        *eeprom_data_size = sizeof(swift_flood_module_eeprom_data_t);
    }
    else {
        *eeprom_data_size = sizeof(swift_eeprom_v2_data_t);
    }

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

int Misc_Device::read_dtof_runtime_status_param(struct adaps_dtof_runtime_status_param **status_param)
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
        *status_param = &last_runtime_status_param;
    }

    return ret;
}

int Misc_Device::get_dtof_inside_temperature(float *temperature)
{
    int ret = 0;

    *temperature = (float) ((double)last_runtime_status_param.inside_temperature_x100 /(double)100.0f);

    return ret;
}


