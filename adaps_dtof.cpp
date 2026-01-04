#include "adaps_dtof.h"
#include "utils.h"
#include <globalapplication.h>

ADAPS_DTOF::ADAPS_DTOF(struct sensor_params params)
{
    memset(depthOutputs, 0, sizeof(depthOutputs));
    memset(&depthInput, 0, sizeof(depthInput));
    memset(&depthConfig, 0, sizeof(depthConfig));
    memcpy(&m_sns_param, &params, sizeof(struct sensor_params));

    m_basic_colors[4] = {255, 0, 0};
    m_basic_colors[3] = {255, 255, 0};
    m_basic_colors[2] = {0, 255, 0};
    m_basic_colors[1] = {0, 255, 255};
    m_basic_colors[0] = {0, 0, 255};

    m_LimitedMaxDistance = COLOR_MAP_HIGH;
    m_rangeHigh = COLOR_MAP_HIGH;
    m_rangeLow = RANGE_MIN;
    m_conversionLibInited = false;
    m_input_frame_cnt = 0;
    m_output_frame_cnt = 0;
    copied_roisram_4_anchorX = NULL_POINTER;
    p_misc_device = qApp->get_misc_dev_instance();
    init_frame_checker(&flc);
    skip_frame_decode = Utils::is_env_var_true(ENV_VAR_SKIP_FRAME_DECODE);
}

ADAPS_DTOF::~ADAPS_DTOF()
{
    DBG_NOTICE("------decode statistics------work_mode: %d, m_input_frame_cnt: %d, m_output_frame_cnt: %d, decoded_rate: %d, flc.total_frames: %d, flc.dropped_frames: %d---\n",
        set_param.work_mode, m_input_frame_cnt, m_output_frame_cnt, m_input_frame_cnt/m_output_frame_cnt, flc.total_frames, flc.dropped_frames);
    p_misc_device = NULL_POINTER;
    free(copied_roisram_4_anchorX);
    copied_roisram_4_anchorX = NULL_POINTER;
}

int ADAPS_DTOF::dump_frame_headinfo(unsigned int frm_sequence, unsigned char *frm_rawdata, int frm_rawdata_size, enum sensor_workmode swk)
{
    if (frm_rawdata == NULL_POINTER || frm_rawdata_size < 3) {
        return -1;  // Invalid input
    }

    if (WK_DTOF_FHR != swk && WK_DTOF_PHR != swk) {
        return -1;  // Invalid input
    }

#if 1
    DBG_NOTICE("sram_id: %d, zone_id: %d, frame_id: %d, frm_sequence: %d.", frm_rawdata[0], frm_rawdata[1], frm_rawdata[2], frm_sequence);
#else
    Utils *utils;
    utils = new Utils();
    utils->hexdump(frm_rawdata, 256, "first 256 bytes of frame raw data");
    delete utils;
#endif

    return 0;
}

// Initialize the frame loss checker
void ADAPS_DTOF::init_frame_checker(FrameLossChecker *checker)
{
    checker->last_id = 0;
    checker->first_frame = 1;
    checker->total_frames = 0;
    checker->dropped_frames = 0;
}

// Check if frames are dropped for the new frame
// Returns: Number of dropped frames (0 = no loss, >0 = number of dropped frames)
int ADAPS_DTOF::check_frame_loss(FrameLossChecker *checker, const unsigned char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 3) {
        return -1;  // Invalid input
    }

    unsigned char current_id = buffer[2];
    int lost_count = 0;

    if (checker->first_frame) {
        checker->first_frame = 0;
    } else {
        // Calculate the expected ID of the next frame
        unsigned char expected_id = (checker->last_id + 1) & 0xFF;
        
        // Calculate the number of dropped frames
        if (current_id != expected_id) {
            if (current_id > expected_id) {
                lost_count = current_id - expected_id;
            } else {
                lost_count = 256 - (expected_id - current_id);
            }
            checker->dropped_frames += lost_count;
        }
    }

    checker->last_id = current_id;
    checker->total_frames++;
    return lost_count;
}

// Get the frame loss rate (percentage)
float ADAPS_DTOF::get_frame_loss_rate(const FrameLossChecker *checker)
{
    if (checker->total_frames <= 1) {
        return 0.0f;
    }
    return (float)checker->dropped_frames / 
           (float)(checker->total_frames - 1) * 100.0f;
}

u8 ADAPS_DTOF::normalizeGreyscale(u16 range) {
    u16 normalized = range - RANGE_MIN;
    // Clamp to min/max
    normalized = MAX(RANGE_MIN, normalized);
    normalized = MIN(RANGE_MAX, normalized);
    // Normalize to 0 to 255
    normalized = normalized - RANGE_MIN;
    normalized = (normalized * 255) / (RANGE_MAX - RANGE_MIN);
    return (u8) normalized;
}

int ADAPS_DTOF::multipleRoiCoordinatesDumpCheck(uint8_t* multiple_roi_sram_data, u16 length, int outImgWidth, int outImgHeight)
{
    int  i, j;
    uint8_t* base = NULL_POINTER;
    int row,column;
    char lineBuffer[outImgWidth + 1];
    int buf_len = 0;
    int spot_cnt;
    int illegal_spot_cnt;

    if (NULL_POINTER == multiple_roi_sram_data) {
        DBG_ERROR("multiple_roi_sram_data is NULL");
        return -1;
    }

    memset(frameCoordinatesMap, 0, sizeof(u8) * outImgWidth * outImgHeight);

    illegal_spot_cnt = 0;
    base = multiple_roi_sram_data;

    for (i = 0; i < length/2; i++)
    {
        row = *(base + i*2);
        column = *(base + i*2 + 1);

        if (row < 0 || column < 0 || row >= outImgHeight || column >= outImgWidth)
        {
            illegal_spot_cnt++;
            DBG_ERROR("Detect the illegal coordinate(%d, %d), illegal_spot_cnt: %d", column, row, illegal_spot_cnt);
        }

        if (row >= 0 && row < outImgHeight && column >= 0 && column < outImgWidth)
        {
            frameCoordinatesMap[row][column] = frameCoordinatesMap[row][column] + 1;
            if (0 != (row + column) && frameCoordinatesMap[row][column] > 1)
            {
                DBG_ERROR("Detect the duplicated coordinate at roi sram(%d, %d), duplicated times: %d", row, column, frameCoordinatesMap[row][column]);
            }
        }
    }


    spot_cnt = 0;
    for (j = 0; j < outImgHeight; j++)
    {
        memset(lineBuffer, 0, outImgWidth + 1);
        buf_len = 0;

        for (i = 0; i < outImgWidth; i++)
        {
            if (0 == frameCoordinatesMap[j][i])
            {
                buf_len += sprintf(lineBuffer + buf_len, "%c", BLANK_CHAR);
            }
            else {
                buf_len += sprintf(lineBuffer + buf_len, "%c", frameCoordinatesMap[j][i] + '0');
                if (0 != (j + i)) // For ROI SRAM, (0,0) is illegal according to Yangdong.
                {
                    spot_cnt++;
                }
            }
        }
        printf("%s\n", lineBuffer);
    }
    DBG_NOTICE("-----outImgWidth: %d, outImgHeight: %d, length: %d, spot_cnt: %d, illegal_spot_cnt: %d----\n", outImgWidth, outImgHeight, length, spot_cnt, illegal_spot_cnt);

    return 0;
}

int ADAPS_DTOF::roiCoordinatesDumpCheck(uint8_t* roi_sram_data, int outImgWidth, int outImgHeight, int roisram_group_index)
{
    int  i, j;
    uint8_t* base = NULL_POINTER;
    int row,column;
    char lineBuffer[outImgWidth + 1];
    int buf_len = 0;
    int spot_cnt;
    int illegal_spot_cnt;

    if (NULL_POINTER == roi_sram_data) {
        DBG_ERROR( "roi_sram_data is NULL");
        return -1;
    }

    memset(frameCoordinatesMap, 0, sizeof(u8) * outImgWidth * outImgHeight);

    illegal_spot_cnt = 0;

    for (j = 0; j < ZONE_COUNT_PER_SRAM_GROUP; j++)
    {
        base = roi_sram_data + (j * PER_CALIB_SRAM_ZONE_SIZE);

        for (i = 0; i < PER_ZONE_MAX_SPOT_COUNT; i++)
        {
            row = *(base + i*2);
            column = *(base + i*2 + 1);

            if (row < 0 || column < 0 || row >= outImgHeight || column >= outImgWidth)
            {
                illegal_spot_cnt++;
                DBG_ERROR("Detect the illegal coordinate(%d, %d) at roi sram, offset: 0x%x, illegal_spot_cnt: %d", column, row,
                    (j * PER_CALIB_SRAM_ZONE_SIZE) + i*2, illegal_spot_cnt);
            }

            if (row >= 0 && row < outImgHeight && column >= 0 && column < outImgWidth)
            {
                frameCoordinatesMap[row][column] = frameCoordinatesMap[row][column] + 1;
                if (0 != (row + column) && frameCoordinatesMap[row][column] > 1)
                {
                    DBG_ERROR("Detect the duplicated coordinate at roi sram(%d, %d), duplicated times: %d", row, column, frameCoordinatesMap[row][column]);
                }
            }
        }
    }

    spot_cnt = 0;
    for (j = 0; j < outImgHeight; j++)
    {
        memset(lineBuffer, 0, outImgWidth + 1);
        buf_len = 0;

        for (i = 0; i < outImgWidth; i++)
        {
            if (0 == frameCoordinatesMap[j][i])
            {
                buf_len += sprintf(lineBuffer + buf_len, "%c", BLANK_CHAR);
            }
            else {
                buf_len += sprintf(lineBuffer + buf_len, "%c", frameCoordinatesMap[j][i] + '0');
                if (0 != (j + i)) // For ROI SRAM, (0,0) is illegal according to Yangdong.
                {
                    spot_cnt++;
                }
            }
        }
        printf("%s\n", lineBuffer);
    }
    DBG_NOTICE("---ROI SRAM%d--outImgWidth: %d, outImgHeight: %d, spot_cnt: %d, illegal_spot_cnt: %d----\n", roisram_group_index, outImgWidth, outImgHeight, spot_cnt, illegal_spot_cnt);

    return 0;
}

void ADAPS_DTOF::roisram_anchor_preproccess(uint8_t *roisram_buf, uint32_t roisram_buf_size)
{
    // Calculate the top-left corner of the spot.
    // if spod size is 8 * 3, then rowOffset = 1, colOffset = 5.
    u8 rowOffset;
    u8 colOffset;
    uint32_t i;

    qApp->get_anchorOffset(&rowOffset, &colOffset);
    DBG_NOTICE("---anchorX = %d, anchorY = %d---\n", colOffset, rowOffset);

    if ((0 == rowOffset) && (0 == colOffset))
        return; // nothing to do

    for (i = 0; i < roisram_buf_size;) {
      if (roisram_buf[i] > rowOffset) {
        roisram_buf[i] -= rowOffset;
      }
      if (roisram_buf[i + 1] > colOffset) {
        roisram_buf[i + 1] -= colOffset;
      }
      i += 2;
    }
}

int ADAPS_DTOF::FillSetWrapperParamFromEepromInfo(uint8_t* pEEPROMData, SetWrapperParam* setparam, WrapperDepthInitInputParams* initInputParams)
{
    int result = 0;
    Utils *utils;
    int dump_roi_sram_size = Utils::get_env_var_intvalue(ENV_VAR_DUMP_ROI_SRAM_SIZE);
    uint32_t dump_offsetdata_param_cnt = Utils::get_env_var_intvalue(ENV_VAR_DUMP_OFFSETDATA_PARAM_COUNT);
    uint32_t dump_walkerror_param_cnt = Utils::get_env_var_intvalue(ENV_VAR_DUMP_WALKERROR_PARAM_COUNT);
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    Host_Communication *host_comm = Host_Communication::getInstance();
    reference_distance_data_param_t* pRefDistanceParam = NULL_POINTER;
    lens_intrinsic_data_param_t* pLensIntrinsicParam = NULL_POINTER;
#endif

    if (NULL_POINTER == pEEPROMData) {
        DBG_ERROR( "pEEPROMData is NULL for EEPROMInfo");
        return -1;
    }

    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        pLensIntrinsicParam = (lens_intrinsic_data_param_t*) host_comm->get_loaded_lens_intrinsic_data();
    }

    if (NULL_POINTER != pLensIntrinsicParam) {
        setparam->adapsLensIntrinsicData  = pLensIntrinsicParam->intrinsic;
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            setparam->adapsLensIntrinsicData  = reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_INTRINSIC_OFFSET);
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type())
        {
            setparam->adapsLensIntrinsicData  = reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_INTRINSIC_OFFSET);
        }
        else {
            setparam->adapsLensIntrinsicData  = reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_INTRINSIC_OFFSET);
        }
    }

    if (Utils::is_env_var_true(ENV_VAR_DUMP_LENS_INTRINSIC))
    {
        for (int i = 0; i < 9; i++)
        {
            DBG_NOTICE("intrinsic Parameters[%d] = %f", i, setparam->adapsLensIntrinsicData[i]);
        }
    }

    loaded_roi_sram_data = qApp->get_mmap_address_4_loaded_roisram();
    loaded_roi_sram_size = qApp->get_size_4_loaded_roisram();

    if (0 != loaded_roi_sram_size && NULL != loaded_roi_sram_data)
    {
        setparam->spot_cali_data = (uint8_t* ) loaded_roi_sram_data;
        initInputParams->pRawData = set_param.spot_cali_data;
        initInputParams->rawDataSize = loaded_roi_sram_size;

        if (true == Utils::is_env_var_true(ENV_VAR_ROI_SRAM_COORDINATES_CHECK))
        {
            multipleRoiCoordinatesDumpCheck(setparam->spot_cali_data, loaded_roi_sram_size, m_sns_param.out_frm_width, m_sns_param.out_frm_height);
        }
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            if (NULL_POINTER == copied_roisram_4_anchorX)
            {
                copied_roisram_4_anchorX = (u8 *) malloc(ADS6401_EEPROM_ROISRAM_DATA_SIZE);
                if (NULL_POINTER == copied_roisram_4_anchorX) {
                    DBG_ERROR("Fail to malloc for copied_roisram_4_anchorX.\n");
                    return -1;
                }
            }

            // create another buffer copy for anchorX operation to keep the original mmaped ROI sram data not be changed.
            initInputParams->rawDataSize = PER_ROISRAM_GROUP_SIZE; // ADS6401_EEPROM_ROISRAM_DATA_SIZE;

            memcpy(copied_roisram_4_anchorX, pEEPROMData + ADS6401_EEPROM_ROISRAM_DATA_OFFSET, initInputParams->rawDataSize);
            setparam->spot_cali_data = copied_roisram_4_anchorX;
            initInputParams->pRawData = set_param.spot_cali_data;
            roisram_anchor_preproccess(setparam->spot_cali_data, initInputParams->rawDataSize);
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type())
        {
            int force_roi_sram_size = Utils::get_env_var_intvalue(ENV_VAR_FORCE_ROI_SRAM_SIZE);
            if (force_roi_sram_size)
            {
                initInputParams->rawDataSize = force_roi_sram_size;
            }
            else {
                initInputParams->rawDataSize = PER_ROISRAM_GROUP_SIZE; // FLOOD_EEPROM_ROISRAM_DATA_SIZE;
            }

            setparam->spot_cali_data          = pEEPROMData + FLOOD_EEPROM_ROISRAM_DATA_OFFSET;
            initInputParams->pRawData = set_param.spot_cali_data;
        }
        else
        {
            swift_eeprom_v2_data_t *p_bigfov_module_eeprom = (swift_eeprom_v2_data_t *) pEEPROMData;
            initInputParams->rawDataSize = p_bigfov_module_eeprom->real_spot_zone_count * PER_CALIB_SRAM_ZONE_SIZE;

            setparam->spot_cali_data          = pEEPROMData + BIG_FOV_MODULE_EEPROM_ROISRAM_DATA_OFFSET;
            initInputParams->pRawData = set_param.spot_cali_data;
        }

        if (true == Utils::is_env_var_true(ENV_VAR_ROI_SRAM_COORDINATES_CHECK))
        {
            roiCoordinatesDumpCheck(setparam->spot_cali_data, m_sns_param.out_frm_width, m_sns_param.out_frm_height, 0);
            if (initInputParams->rawDataSize >= (2 * PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP))
            {
                roiCoordinatesDumpCheck(setparam->spot_cali_data + PER_CALIB_SRAM_ZONE_SIZE * ZONE_COUNT_PER_SRAM_GROUP, m_sns_param.out_frm_width, m_sns_param.out_frm_height, 1);
            }
        }
    }

    if (dump_roi_sram_size)
    {
        utils = new Utils();
        utils->hexdump(setparam->spot_cali_data, dump_roi_sram_size, "ROI SRAM data dump to Algo lib");
        delete utils;
    }

    setparam->adapsSpodOffsetData     = qApp->get_loaded_spotoffset_data();
#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    setparam->adapsSpodOffsetDataLength = qApp->get_loaded_spotoffset_data_size();

    if (0 != setparam->adapsSpodOffsetDataLength && NULL != setparam->adapsSpodOffsetData)
    {
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_SPOTOFFSET_OFFSET);
            setparam->adapsSpodOffsetDataLength = ADS6401_EEPROM_SPOTOFFSET_SIZE;
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type())
        {
            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_SPOTOFFSET_OFFSET + FLOOD_ONE_SPOD_OFFSET_BYTE_SIZE);  //pointer to offset0
            setparam->adapsSpodOffsetDataLength = FLOOD_EEPROM_SPOTOFFSET_SIZE;
        }
        else
        {
            swift_eeprom_v2_data_t *p_bigfov_module_eeprom = (swift_eeprom_v2_data_t *) pEEPROMData;

            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_SPOTOFFSET_OFFSET);
            setparam->adapsSpodOffsetDataLength = p_bigfov_module_eeprom->real_spot_zone_count * PER_ZONE_MAX_SPOT_COUNT * sizeof(float);
        }
    }
#else
    if (NULL != setparam->adapsSpodOffsetData)
    {
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_SPOTOFFSET_OFFSET);
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type())
        {
            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_SPOTOFFSET_OFFSET + FLOOD_ONE_SPOD_OFFSET_BYTE_SIZE);  //pointer to offset0
        }
        else
        {
            setparam->adapsSpodOffsetData     = reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_SPOTOFFSET_OFFSET);
        }
    }
#endif

    if (dump_offsetdata_param_cnt > 0 && NULL != setparam->adapsSpodOffsetData)
    {
        uint32_t i;

        DBG_PRINTK("First %d offset data dump to Algo lib\n", dump_offsetdata_param_cnt);
        for (i = 0; i < dump_offsetdata_param_cnt; i++)
        {
            DBG_PRINTK("   %4d      %f\n", i, setparam->adapsSpodOffsetData[i]);
        }
    }

    if (ADS6401_MODULE_SPOT == qApp->get_module_type())
    {
        setparam->proximity_hist          = pEEPROMData + ADS6401_EEPROM_PROX_HISTOGRAM_OFFSET;
        //setparam->accurateSpotPosData     = reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_ACCURATESPODPOS_OFFSET);
    }
    else {
        setparam->proximity_hist          = NULL_POINTER;
        //setparam->accurateSpotPosData     = NULL_POINTER;
    }

    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        pRefDistanceParam = (reference_distance_data_param_t*) host_comm->get_loaded_ref_distance_data();
    }

    if (NULL_POINTER != pRefDistanceParam) {
        setparam->cali_ref_tempe[AdapsEnvTypeIndoor - 1]  = pRefDistanceParam->indoorCalibTemperature;
        setparam->cali_ref_tempe[AdapsEnvTypeOutdoor - 1] = pRefDistanceParam->outdoorCalibTemperature;
        setparam->cali_ref_depth[AdapsEnvTypeIndoor - 1]  = pRefDistanceParam->indoorCalibRefDistance;
        setparam->cali_ref_depth[AdapsEnvTypeOutdoor - 1] = pRefDistanceParam->outdoorCalibRefDistance;
    }
    else {
        if (ADS6401_MODULE_SPOT == qApp->get_module_type())
        {
            setparam->cali_ref_tempe[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_tempe[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_OUTDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_INDOOR_CALIBREFDISTANCE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + ADS6401_EEPROM_OUTDOOR_CALIBREFDISTANCE_OFFSET);
        }
        else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type())
        {
            setparam->cali_ref_tempe[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_tempe[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_OUTDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_INDOOR_CALIBREFDISTANCE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + FLOOD_EEPROM_OUTDOOR_CALIBREFDISTANCE_OFFSET);
        }
        else
        {
            setparam->cali_ref_tempe[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_INDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_tempe[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_OUTDOOR_CALIBTEMPERATURE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeIndoor - 1]  = *reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_INDOOR_CALIBREFDISTANCE_OFFSET);
            setparam->cali_ref_depth[AdapsEnvTypeOutdoor - 1] = *reinterpret_cast<float*>(pEEPROMData + BIG_FOV_MODULE_EEPROM_OUTDOOR_CALIBREFDISTANCE_OFFSET);
        }
    }

    if (Utils::is_env_var_true(ENV_VAR_DUMP_CALIB_REFERENCE_DISTANCE))
    {
        DBG_PRINTK("indoorCalibTemperature = %f", setparam->cali_ref_tempe[AdapsEnvTypeIndoor - 1]);
        DBG_PRINTK("indoorCalibRefDistance = %f", setparam->cali_ref_depth[AdapsEnvTypeIndoor - 1]);

        DBG_PRINTK("outdoorCalibTemperature = %f", setparam->cali_ref_tempe[AdapsEnvTypeOutdoor - 1]);
        DBG_PRINTK("outdoorCalibRefDistance = %f", setparam->cali_ref_depth[AdapsEnvTypeOutdoor - 1]);
    }

#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        set_param.walkerror = (0 != host_comm->get_req_walkerror_enable());
    }
    else 
#endif
    {
        set_param.walkerror = true;
    }

    if (true == set_param.walkerror)
    {
        if (true == Utils::is_env_var_true(ENV_VAR_DISABLE_WALK_ERROR))
        {
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
            setparam->walkerror = false;
            setparam->walk_error_para_list = NULL_POINTER;
#else
            setparam->walkerror = false;
            setparam->calibrationInfo = NULL_POINTER;
            setparam->walk_error_para_list = NULL_POINTER;
            setparam->walk_error_para_list_length = 0;
#endif
            DBG_NOTICE("force walk error to FALSE by environment variable 'disable_walk_error'.");
        }
        else {
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
            setparam->walk_error_para_list = qApp->get_loaded_walkerror_data();
#else
            setparam->calibrationInfo = NULL_POINTER;
            setparam->walk_error_para_list = qApp->get_loaded_walkerror_data();
            setparam->walk_error_para_list_length = qApp->get_loaded_walkerror_data_size();
#endif

            if (
#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
                0 != setparam->walk_error_para_list_length && 
#endif
                NULL != setparam->walk_error_para_list)
            {
                DBG_NOTICE("walk error to true since walkerror parameter is loaded.");
                setparam->walkerror = true;
            }
            else {
                if (ADS6401_MODULE_SPOT == qApp->get_module_type())
                {
                    DBG_NOTICE("walk error to true since walkerror parameter exist in eeprom.");
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
                    setparam->walkerror = true;
                    setparam->walk_error_para_list = pEEPROMData + ADS6401_EEPROM_WALK_ERROR_OFFSET;
#else
                    setparam->walkerror = true;
                    setparam->calibrationInfo = pEEPROMData + ADS6401_EEPROM_VERSION_INFO_OFFSET;
                    setparam->walk_error_para_list = pEEPROMData + ADS6401_EEPROM_WALK_ERROR_OFFSET;
                    setparam->walk_error_para_list_length = ADS6401_EEPROM_WALK_ERROR_SIZE;
#endif
                }
                else if (ADS6401_MODULE_SMALL_FLOOD == qApp->get_module_type()) {
                    // PAY ATTETION: No walk error parameters in eeprom for flood module
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
                        setparam->walkerror = false;
                        setparam->walk_error_para_list = NULL_POINTER;
#else
                    setparam->walkerror = false;
                    setparam->calibrationInfo = NULL_POINTER;
                    setparam->walk_error_para_list = NULL_POINTER;
                    setparam->walk_error_para_list_length = 0;
#endif
                    DBG_NOTICE("force walk error to FALSE since swift flood module has no walkerror parameters in eeprom.");
                }
                else {
#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
                    setparam->walkerror = true;
                    setparam->walk_error_para_list = pEEPROMData + BIG_FOV_MODULE_EEPROM_WALK_ERROR_OFFSET;
#else
                    swift_eeprom_v2_data_t *p_bigfov_module_eeprom = (swift_eeprom_v2_data_t *) pEEPROMData;

                    setparam->walkerror = true;
                    setparam->calibrationInfo = pEEPROMData + BIG_FOV_MODULE_EEPROM_CALIBRATIONINFO_OFFSET;
                    setparam->walk_error_para_list = pEEPROMData + BIG_FOV_MODULE_EEPROM_WALK_ERROR_OFFSET;
                    setparam->walk_error_para_list_length = p_bigfov_module_eeprom->real_spot_zone_count * PER_ZONE_MAX_SPOT_COUNT * sizeof(WalkErrorParam_t);
#endif
                }
            }
    
            if (dump_walkerror_param_cnt > 0 && NULL != setparam->walk_error_para_list)
            {
                uint32_t i;
                WalkErrorParam_t *pWalkErrParam = (WalkErrorParam_t *) setparam->walk_error_para_list;
            
                DBG_PRINTK("ParamIndex zoneId   spotId      x    y    paramD        paramX      paramY      paramZ      param0    sizeof(WalkErrorParam_t): %ld\n", sizeof(WalkErrorParam_t));
                for (i = 0; i < dump_walkerror_param_cnt; i++)
                {
                    DBG_PRINTK("   %4d      %d      %4d     <%3d, %3d>  %f     %f    %f    %f    %f\n", 
                        i, pWalkErrParam[i].zoneId, pWalkErrParam[i].spotId, pWalkErrParam[i].x, pWalkErrParam[i].y,
                        pWalkErrParam[i].paramD, pWalkErrParam[i].paramX, pWalkErrParam[i].paramY, pWalkErrParam[i].paramZ, pWalkErrParam[i].param0);
                }
            }
        }
    }

    qApp->set_walkerror_enable(setparam->walkerror); // set initial value for application CLASS.

#ifdef WINDOWS_BUILD
    setparam->dump_data =false;
#endif

    return result;
}

int ADAPS_DTOF::initParams(WrapperDepthInitInputParams* initInputParams, WrapperDepthInitOutputParams* initOutputParams)
{
    struct adaps_dtof_exposure_param *p_exposure_param;
#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    Host_Communication *host_comm = Host_Communication::getInstance();
#endif

    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL");
        return 0 - __LINE__;
    }

    memset(&set_param, 0, sizeof(SetWrapperParam));
    set_param.work_mode = static_cast<int>(m_sns_param.work_mode);

#if !defined(STANDALONE_APP_WITHOUT_HOST_COMMUNICATION)
    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        set_param.expand_pixel = host_comm->get_req_expand_pixel();
    }
    else {
        set_param.expand_pixel = false;
    }

    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        set_param.compose_subframe = host_comm->get_req_compose_subframe();
    }
    else {
        set_param.compose_subframe = true;
    }

    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        set_param.mirror_frame.mirror_x = host_comm->get_req_mirror_x();
    }
    else {
        set_param.mirror_frame.mirror_x = false;
    }

    if (true == qApp->is_capture_req_from_host() && NULL_POINTER != host_comm)
    {
        set_param.mirror_frame.mirror_y = host_comm->get_req_mirror_y();
    }
    else {
        set_param.mirror_frame.mirror_y = false;
    }
#else
    set_param.expand_pixel = false;
    set_param.compose_subframe = true;
    set_param.mirror_frame.mirror_x = false;
    set_param.mirror_frame.mirror_y = false;
#endif

#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 6, 5)
    set_param.moduleKernelType = qApp->get_module_kernel_type();
#endif

    // always allow to check environment variable to change these setting for debug.
    if (true == Utils::is_env_var_true(ENV_VAR_ENABLE_EXPAND_PIXEL))
    {
        set_param.expand_pixel = true;
    }

    if (true == Utils::is_env_var_true(ENV_VAR_DISABLE_COMPOSE_SUBFRAME))
    {
        set_param.compose_subframe = false;
    }

    if (true == Utils::is_env_var_true(ENV_VAR_MIRROR_X_ENABLE))
    {
        set_param.mirror_frame.mirror_x = true;
    }

    if (true == Utils::is_env_var_true(ENV_VAR_MIRROR_Y_ENABLE))
    {
        set_param.mirror_frame.mirror_y = true;
    }

    if (true == Utils::is_env_var_true(ENV_VAR_DISABLE_WALK_ERROR))
    {
        set_param.walkerror = false;
    }

    set_param.peak_index = 1;
    set_param.env_type = m_sns_param.env_type;
    set_param.measure_type = m_sns_param.measure_type;

    DepthMapWrapperGetVersion(m_DepthLibversion);
    set_param.OutAlgoVersion = (uint8_t*)m_DepthLibversion;

    p_eeprominfo = p_misc_device->get_dtof_calib_eeprom_param();
    if (NULL_POINTER == p_eeprominfo) {
        DBG_ERROR("p_eeprominfo is NULL for EEPROMInfo");
        return 0 - __LINE__;
    }
    FillSetWrapperParamFromEepromInfo((uint8_t* ) p_eeprominfo, &set_param, initInputParams);

    p_exposure_param = (struct adaps_dtof_exposure_param *) p_misc_device->get_dtof_exposure_param();
    if (NULL_POINTER == p_exposure_param) {
        DBG_ERROR("p_exposure_param is NULL");
        return 0 - __LINE__;
    }
    set_param.exposure_period = p_exposure_param->exposure_period;
    set_param.ptm_fine_exposure_value = p_exposure_param->ptm_fine_exposure_value;

    DBG_INFO("ptm fine exposure value: 0x%x\n", set_param.ptm_fine_exposure_value);
    DBG_INFO("exposure period value: 0x%x\n", set_param.exposure_period);
    DBG_INFO("indoor calib temperature: %f, outdoor: %f\n",
           set_param.cali_ref_tempe[0], set_param.cali_ref_tempe[1]);
    DBG_INFO("indoor calib ref depth: %f, outdoor: %f\n",
           set_param.cali_ref_depth[0], set_param.cali_ref_depth[1]);

    initInputParams->width = m_sns_param.raw_width;
    initInputParams->height = m_sns_param.raw_height;
    initInputParams->dm_width = m_sns_param.out_frm_width;
    initInputParams->dm_height = m_sns_param.out_frm_height;

    initInputParams->configFilePath = m_DepthLibConfigXmlPath;
    sprintf(m_DepthLibConfigXmlPath, "%s", DEPTH_LIB_CONFIG_PATH);
    initInputParams->setparam = set_param;

    initOutputParams->exposure_time = &m_exposure_time;
    initOutputParams->sensitivity = &m_sensitivity;

    if (qApp->is_roi_sram_rolling())
    {
        sub_frame_cnt_per_image_frame = (initInputParams->rawDataSize / PER_ROISRAM_GROUP_SIZE) * 4;
    }
    else {
        sub_frame_cnt_per_image_frame = 4;
    }

#if defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    DBG_NOTICE("initParams success, roi_sram_size: %d, roi_sram_rolling: %d, walkerror: %d, loaded_roi_sram_size: %d, work_mode=%d env_type=%d  measure_type=%d", 
         initInputParams->rawDataSize, qApp->is_roi_sram_rolling(), set_param.walkerror, loaded_roi_sram_size, set_param.work_mode,set_param.env_type,set_param.measure_type);
#else
    DBG_NOTICE("initParams success, roi_sram_size: %d, roi_sram_rolling: %d, walkerror: %d, loaded_roi_sram_size: %d, SpodOffsetDataLength: %d, walk_error_data_size: %d, work_mode=%d env_type=%d  measure_type=%d, moduleKernelType: %d", 
        initInputParams->rawDataSize, qApp->is_roi_sram_rolling(), set_param.walkerror, loaded_roi_sram_size, set_param.adapsSpodOffsetDataLength, set_param.walk_error_para_list_length, set_param.work_mode,set_param.env_type,set_param.measure_type, set_param.moduleKernelType);
#endif

    return 0;
}

int ADAPS_DTOF::adaps_dtof_initilize()
{
    int result = 0;
    WrapperDepthInitInputParams     initInputParams                 = {};
    WrapperDepthInitOutputParams    initOutputParams;

    result = initParams(&initInputParams,&initOutputParams);
    if (result < 0) {
        DBG_ERROR("Fail to initParams, ret: %d", result);
        return result;
    }

    result = DepthMapWrapperCreate(&m_handlerDepthLib, initInputParams, initOutputParams);
    if (!m_handlerDepthLib || result < 0) {
        DBG_ERROR("Error creating depth map wrapper, result: %d, m_handlerDepthLib: %p", result, m_handlerDepthLib);
        return result;
    }

#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
    CircleForMask circleForMask;
    circleForMask.CircleMaskCenterX = m_sns_param.out_frm_width;
    circleForMask.CircleMaskCenterY = m_sns_param.out_frm_height;
    circleForMask.CircleMaskR = 0.0f;

    DepthMapWrapperSetCircleMask(m_handlerDepthLib,circleForMask);
#endif

    DBG_NOTICE("Adaps depth lib init done, expected lib version(api interface): v%d.%d.%d, current running lib(.so): v%s, m_handlerDepthLib: %p.",
        ALGO_LIB_VERSION_MAJOR, ALGO_LIB_VERSION_MINOR, ALGO_LIB_VERSION_REVISION, m_DepthLibversion,
        m_handlerDepthLib);

    m_conversionLibInited = true;

    return result;
}

void ADAPS_DTOF::adaps_dtof_release()
{
    if (NULL_POINTER != m_handlerDepthLib)
    {
        DepthMapWrapperDestroy(m_handlerDepthLib);
        DBG_INFO("Adaps depth lib destroy okay.");
        m_handlerDepthLib = NULL_POINTER;
    }
}

void ADAPS_DTOF::PrepareFrameParam(WrapperDepthCamConfig *wrapper_depth_map_config)
{
    float  t = 0.0f;

    if (-1 == p_misc_device->get_dtof_inside_temperature(&t)) {
        DBG_ERROR("Fail to read temperature, errno: %s (%d)...", 
            strerror(errno), errno);
        return;
    }

    if (t < CHIP_TEMPERATURE_MIN_THRESHOLD) {// Eg. m_currToFTemperature in camxsensornode.cpp is not set since temperature read failure.
        DBG_ERROR("too low temperature: %f", t);
        wrapper_depth_map_config->frame_parameters.laser_realtime_tempe = CHIP_TEMPERATURE_MIN_THRESHOLD;
    }
    else if (t > CHIP_TEMPERATURE_MAX_THRESHOLD) {// Eg. m_currToFTemperature in camxsensornode.cpp is not set since temperature read failure.
        DBG_ERROR("too high temperature: %f", t);
        wrapper_depth_map_config->frame_parameters.laser_realtime_tempe = CHIP_TEMPERATURE_MAX_THRESHOLD;
    }
    else {
        wrapper_depth_map_config->frame_parameters.laser_realtime_tempe = t;
    }
    //wrapper_depth_map_config->frame_parameters.laser_realtime_tempe = CHIP_TEMPERATURE_MAX_THRESHOLD;

    //DBG_INFO( "adapsChipTemperature: %f\n " ,wrapper_depth_map_config->frame_parameters.laser_realtime_tempe);

    wrapper_depth_map_config->frame_parameters.measure_type_in = (AdapsMeasurementType) m_sns_param.measure_type;
    //DBG_INFO("AdapsMeasurementType: %d \n" , wrapper_depth_map_config->frame_parameters.measure_type_in);

//    wrapper_depth_map_config->frame_parameters.focutPoint[0] = 1;
//    wrapper_depth_map_config->frame_parameters.focutPoint[1] = 2; //need to get this value
    /*DBG_INFO("Adaps FocusPoint:x %d y %d \n", 
                wrapper_depth_map_config->frame_parameters.focutPoint[0],
                wrapper_depth_map_config->frame_parameters.focutPoint[1]);*/

    wrapper_depth_map_config->frame_parameters.env_type_in = (AdapsEnvironmentType) m_sns_param.env_type;
   // DBG_INFO("AdapsEnvironmentType: %d \n" , wrapper_depth_map_config->frame_parameters.env_type_in);

    wrapper_depth_map_config->frame_parameters.advised_env_type_out     = &m_sns_param.advisedEnvType;
    wrapper_depth_map_config->frame_parameters.advised_measure_type_out = &m_sns_param.advisedMeasureType;

    wrapper_depth_map_config->frame_parameters.focusRoi.FocusLeftTopX = 0;
    wrapper_depth_map_config->frame_parameters.focusRoi.FocusLeftTopY = 0;
    wrapper_depth_map_config->frame_parameters.focusRoi.FocusRightBottomX = m_sns_param.out_frm_width;
    wrapper_depth_map_config->frame_parameters.focusRoi.FocusRightBottomY = m_sns_param.out_frm_height;

    //wrapper_depth_map_config->frame_parameters.walkerror_enable = qApp->is_walkerror_enabled();
}

void ADAPS_DTOF::Distance_2_BGRColor(int bucketNum, float bucketSize, u16 distance, struct BGRColor *destColor)
{
    float scale = (float)((distance -  bucketNum * bucketSize) / bucketSize);
    destColor->Blue = (u8)(m_basic_colors[bucketNum].Blue + (m_basic_colors[bucketNum+1].Blue - m_basic_colors[bucketNum].Blue) * scale);
    destColor->Green = (u8)(m_basic_colors[bucketNum].Green + (m_basic_colors[bucketNum+1].Green - m_basic_colors[bucketNum].Green) * scale);
    destColor->Red = (u8)(m_basic_colors[bucketNum].Red + (m_basic_colors[bucketNum+1].Red - m_basic_colors[bucketNum].Red) * scale);
}

int ADAPS_DTOF::dumpSpotCount(const u16 depth16_buffer[], const int outImgWidth, const int outImgHeight, const uint32_t frm_sequence, const uint32_t out_frame_cnt, int decodeRet, int callline)
{
    int index;
    int  i, j;
    u16 depth16;
    u16 distance;
    u8 confidence;
    int spot_cnt_4_high_confidence = 0;
    int spot_cnt_4_mid_confidence = 0;
    int spot_cnt_4_low_confidence = 0;
    int spot_cnt_4_other_confidence = 0;

    if (NULL_POINTER == depth16_buffer) {
        DBG_ERROR( "depth16_buffer is NULL");
        return -1;
    }

    for (j = 0; j < outImgHeight; j++)
    {
        for (i = 0; i < outImgWidth; i++)
        {
            index = j * outImgWidth + i;
            depth16 = depth16_buffer[index];
            distance = depth16 & DEPTH_MASK;
            confidence = (depth16 >> DEPTH_BITS) & CONFIDENCE_MASK;

            if (0 != distance)
            {
                switch (confidence)
                {
                    case ANDROID_CONF_HIGH:
                        spot_cnt_4_high_confidence++;
                        break;

                    case ANDROID_CONF_MEDIUM:
                        spot_cnt_4_mid_confidence++;
                        break;

                    case ANDROID_CONF_LOW:
                        spot_cnt_4_low_confidence++;
                        break;

                    default:
                        spot_cnt_4_other_confidence++;
                        break;
                }
            }
        }
    }

    DBG_NOTICE("### output_frame[%d] has (%d + %d + %d + %d) = %d spots from high to low confidence, frm_sequence: %d, decodeRet: %d, call from line: %d.", 
        out_frame_cnt,
        spot_cnt_4_high_confidence, spot_cnt_4_mid_confidence, spot_cnt_4_other_confidence, spot_cnt_4_low_confidence,
        spot_cnt_4_high_confidence + spot_cnt_4_mid_confidence + spot_cnt_4_other_confidence + spot_cnt_4_low_confidence,
        frm_sequence, decodeRet, callline);

    return spot_cnt_4_high_confidence;
}

int ADAPS_DTOF::depthMapDump(const u16 depth16_buffer[], const int outImgWidth, const int outImgHeight, const uint32_t out_frame_cnt, int callline)
{
    int index;
    int  i, j;
    u8 confidence;
    u16 distance;
    char lineBuffer[outImgWidth + 1];
    int buf_len = 0;
    int non_zero_spot_count = 0;
    int h_conf_spot_count = 0;
    int l_conf_spot_count = 0;
    int frm_interval_4_depthmap_dump = 0;

    if (NULL_POINTER == depth16_buffer) {
        DBG_ERROR( "depth16_buffer is NULL");
        return -1;
    }

    frm_interval_4_depthmap_dump = Utils::get_env_var_intvalue(ENV_VAR_DUMP_DEPTH_MAP_FRAME_ITVL);

    if (0 == frm_interval_4_depthmap_dump)
        return 0;

    if (0 != (out_frame_cnt % frm_interval_4_depthmap_dump))
        return 0;

    for (j = 0; j < outImgHeight; j++)
    {
        memset(lineBuffer, 0, outImgWidth + 1);
        buf_len = 0;

        for (i = 0; i < outImgWidth; i++)
        {
            index = j * outImgWidth + i;
            distance = depth16_buffer[index] & DEPTH_MASK;

            if ((0 != distance) && (0 != (j + i)))  // For ROI SRAM, (0,0) is illegal according to Yangdong.
            {
                confidence = (depth16_buffer[index] >> DEPTH_BITS) & CONFIDENCE_MASK;

                switch (confidence)
                {
                    case ANDROID_CONF_HIGH:
                         buf_len += sprintf(lineBuffer + buf_len, "%c", 'H');
                         h_conf_spot_count++;
                         break;

                    case ANDROID_CONF_LOW:
                         buf_len += sprintf(lineBuffer + buf_len, "%c", 'L');
                         l_conf_spot_count++;
                         break;

                    default:
                        buf_len += sprintf(lineBuffer + buf_len, "%c", 'm');
                        break;
                }

                non_zero_spot_count++;
            }
            else {
                buf_len += sprintf(lineBuffer + buf_len, "%c", BLANK_CHAR);
            }
        }

        printf("%s\n", lineBuffer);
    }

    DBG_NOTICE("-----out_frame_cnt: %d, non_zero_spot_count: %d, h_conf_spot_count: %d, l_conf_spot_count: %d,frm_interval_4_depthmap_dump: %d call from line: %d----\n",
        out_frame_cnt, non_zero_spot_count, h_conf_spot_count, l_conf_spot_count, frm_interval_4_depthmap_dump, callline);

    return 0;
}

void ADAPS_DTOF::GetDepth4watchSpot(const u16 depth16_buffer[], const int outImgWidth, u8 x, u8 y, u16 *distance, u8 *confidence)
{
    int rawImgIdx;
    rawImgIdx = y * outImgWidth + x;

    *distance = depth16_buffer[rawImgIdx] & DEPTH_MASK;
    *confidence = (depth16_buffer[rawImgIdx] >> DEPTH_BITS) & CONFIDENCE_MASK;
}

void ADAPS_DTOF::ConvertDepthToColoredMap(const u16 depth16_buffer[], u8 depth_colored_map[], u8 depth_confidence_map[], const int outImgWidth, const int outImgHeight)
{
    int rawImgIdx;
    struct BGRColor bgrColor;
    u16 distance;
    u8 confidence;
    float bucketSize;
    int bucketNum;
    int  i, j;
    int non_zero_depth_count = 0;

    for (j = 0; j < outImgHeight; j++) {
        for (i = 0; i < outImgWidth; i++) {
            rawImgIdx = j * outImgWidth + i;
            distance = depth16_buffer[rawImgIdx] & DEPTH_MASK;
            confidence = (depth16_buffer[rawImgIdx] >> DEPTH_BITS) & CONFIDENCE_MASK;

#if defined(COLOR_MAP_SYNCED_WITH_PC_SPADISAPP)
                double maxDepth = 30.0; // private static double maxDepth = 30.0; from line of SpadisPC\spadisApp\Models\GenerateColorMap.cs
                double distance_db = (double) (distance / 1000.0);

                float rangeLow = qApp->get_RealDistanceMinMappedRange();
                float rangeHigh = qApp->get_RealDistanceMaxMappedRange();

                if (distance_db > maxDepth) distance_db = maxDepth; // If the value is out of the range
                if (distance_db < 0.0) distance_db = 0.0;

                bucketSize = (rangeHigh - rangeLow) / 4.0f;
                if (bucketSize <= 0.001f)
                {
                    bgrColor.Red = m_basic_colors[0].Red;
                    bgrColor.Green = m_basic_colors[0].Green;
                    bgrColor.Blue = m_basic_colors[0].Blue;
                }
                else
                {
                    // Seperate 1023 in 4 buckets, each one has 256 number
                    bucketNum = (int)((distance_db - rangeLow) / bucketSize);

                    if (distance_db > rangeLow && bucketNum <= 3)
                    {
                        Distance_2_BGRColor(bucketNum, bucketSize, (distance_db - m_rangeLow), &bgrColor);
                    }
                    else if (distance_db == 0.0) // Show as black
                    {
                        bgrColor.Red = 0;
                        bgrColor.Green = 0;
                        bgrColor.Blue = 0;
                    }
                    else if (distance_db <= rangeLow) // Show as colors[0]
                    {
                        bgrColor.Red = m_basic_colors[0].Red;
                        bgrColor.Green = m_basic_colors[0].Green;
                        bgrColor.Blue = m_basic_colors[0].Blue;
                    }
                    else // Show as colors[4]
                    {
                        bgrColor.Red = m_basic_colors[4].Red;
                        bgrColor.Green = m_basic_colors[4].Green;
                        bgrColor.Blue = m_basic_colors[4].Blue;
                    }
                }
#else
            if (distance > m_LimitedMaxDistance)
            {
                distance = m_LimitedMaxDistance; // If the value is out of the range
            }

            bucketSize = (m_rangeHigh - m_rangeLow) / 4.0f;
            bucketNum = (int)((distance - m_rangeLow) / bucketSize);

            if (distance > m_rangeLow && bucketNum <= 3)
            {
                Distance_2_BGRColor(bucketNum, bucketSize, (distance - m_rangeLow), &bgrColor);
            }
            else if (distance == 0) { // Show as black
                bgrColor.Red = 0;
                bgrColor.Green = 0;
                bgrColor.Blue = 0;
            }
            else if (distance <= m_rangeLow)
            { // Show as colors[0]
                bgrColor.Red = m_basic_colors[0].Red;
                bgrColor.Green = m_basic_colors[0].Green;
                bgrColor.Blue = m_basic_colors[0].Blue;

                if (Utils::is_env_var_true(ENV_VAR_DUMP_SPOT_DEPTH))
                {
                    DBG_NOTICE("### frm_cnt: %d,Spot(%d, %d) distance: %d mm, confidence: %d, non_zero_depth_count: %d, bgrColor(%d, %d, %d)",
                        m_input_frame_cnt, i, j, distance, confidence, non_zero_depth_count, bgrColor.Blue, bgrColor.Green, bgrColor.Red);
                }
            }
            else { // Show as colors[4]
                bgrColor.Red = m_basic_colors[4].Red;
                bgrColor.Green = m_basic_colors[4].Green;
                bgrColor.Blue = m_basic_colors[4].Blue;

                if (Utils::is_env_var_true(ENV_VAR_DUMP_SPOT_DEPTH))
                {
                    DBG_NOTICE("--- frm_cnt: %d, Spot(%d, %d) distance: %d mm, confidence: %d, non_zero_depth_count: %d", m_input_frame_cnt, i, j, distance, confidence, non_zero_depth_count);
                }
            }
#endif

            depth_colored_map[rawImgIdx * 3 + 0] = bgrColor.Red;
            depth_colored_map[rawImgIdx * 3 + 1] = bgrColor.Green;
            depth_colored_map[rawImgIdx * 3 + 2] = bgrColor.Blue;

            if (0 != distance)
            {
                non_zero_depth_count++;

                if (ANDROID_CONF_LOW == confidence)
                {
                    depth_confidence_map[rawImgIdx * 3 + 0] = 0xFF; //Red;
                    depth_confidence_map[rawImgIdx * 3 + 1] = 0x00; //Green;
                    depth_confidence_map[rawImgIdx * 3 + 2] = 0x00; //Blue;
                }
                else if (ANDROID_CONF_HIGH == confidence)
                {
                    depth_confidence_map[rawImgIdx * 3 + 0] = 0x00; //Red;
                    depth_confidence_map[rawImgIdx * 3 + 1] = 0xFF; //Green;
                    depth_confidence_map[rawImgIdx * 3 + 2] = 0x00; //Blue;
                }
                else {
                    if (Utils::is_env_var_true(ENV_VAR_DUMP_MID_CONF_ENABLE))
                    {
                        DBG_NOTICE("frm_cnt: %d, non_zero_depth_count: %d, Spot(%d, %d) depth16: 0x%x, distance: %d mm, confidence: %d", 
                            m_input_frame_cnt, non_zero_depth_count, i, j, depth16_buffer[rawImgIdx], distance, confidence);
                    }
                    depth_confidence_map[rawImgIdx * 3 + 0] = 0xFF; //Red;
                    depth_confidence_map[rawImgIdx * 3 + 1] = 0xFF; //Green;
                    depth_confidence_map[rawImgIdx * 3 + 2] = 0x00; //Blue;
                }
            }
            else {
                // show BLACK for those spots whose distance is 0.
                depth_confidence_map[rawImgIdx * 3 + 0] = 0x00; //Red;
                depth_confidence_map[rawImgIdx * 3 + 1] = 0x00; //Green;
                depth_confidence_map[rawImgIdx * 3 + 2] = 0x00; //Blue;
            }
        }
    }

    DBG_INFO("### frame_%d non_zero_depth_count: %d, set_param.expand_pixel: %d", m_input_frame_cnt, non_zero_depth_count, set_param.expand_pixel);
}

void ADAPS_DTOF::ConvertGreyscaleToColoredMap(u16 depth16_buffer[], u8 greyscale_colored_map[], int outImgWidth, int outImgHeight)
{
#if defined(COLOR_MAP_SYNCED_WITH_PC_SPADISAPP)
    int i;
    u16 value;
    double scale;
    double range;
    double imgValue;
    int maxColor = 255;
    u16 GrayScaleMinMappedRange = qApp->get_GrayScaleMinMappedRange();
    u16 GrayScaleMaxMappedRange = qApp->get_GrayScaleMaxMappedRange();

    range = GrayScaleMaxMappedRange - GrayScaleMinMappedRange;
    
    for (i = 0; i < outImgHeight * outImgWidth; i++)
    {
        value = depth16_buffer[i];
        if (value <= GrayScaleMinMappedRange)
        {
            imgValue = 0;
        }
        else if (value >= GrayScaleMaxMappedRange)
        {
            imgValue = maxColor;
        }
        else
        {
            scale = (value - GrayScaleMinMappedRange) / range;
            imgValue = scale * maxColor;
        }
        greyscale_colored_map[i * 3] = (u8)imgValue;
        greyscale_colored_map[i * 3 + 1] = (u8)imgValue;
        greyscale_colored_map[i * 3 + 2] = (u8)imgValue;
    }
#else
    int rawImgIdx;
    int  i, j, rgb_index = 0;
    u16 distance;
    u8 greyColor;

    for (j = 0; j < outImgHeight; j++) {
        for (i = 0; i < outImgWidth; i++) {
            rawImgIdx = j * outImgWidth + i;
            distance = depth16_buffer[rawImgIdx] & DEPTH_MASK;
            greyColor = normalizeGreyscale(distance);

            greyscale_colored_map[rgb_index * 3 + 0] = greyColor;
            greyscale_colored_map[rgb_index * 3 + 1] = greyColor;
            greyscale_colored_map[rgb_index * 3 + 2] = greyColor;
            rgb_index++;

        }
    }
#endif
}

SpotPoint* ADAPS_DTOF::get_spcific_histogram(uint16_t x, uint16_t y)
{
    SpotPoint* spotPoint = NULL_POINTER;
    bool found = false;

    for (int group = 0; group < MAX_SRAM_DATA_NUMBERS ; group++) {
        for (int zone = 0; zone < ZONE_SIZE ; zone++) {
            for (int i = 0; i < SWIFT_SPOT_COUNTS_PER_ZONE; i++) {
                //SpotPoint* spotPoint = *(depthOutputs[0].outAllPointsPtr[0][group][zone] + i);
                SpotPoint* spotPoint = (*depthOutputs[0].outAllPointsPtr)[group][zone][i];
                if ((spotPoint->x == x) && (spotPoint->y == y)) {
                    found = true;
                }
            }
        }
    }

    if (false == found)
    {
        return NULL_POINTER;
    }
    else {
        return spotPoint;
    }
}

int ADAPS_DTOF::dtof_frame_decode(unsigned int frm_sequence, unsigned char *frm_rawdata, int frm_rawdata_size, u16 depth16_buffer[], pc_pkt_t *point_cloud_buffer, enum sensor_workmode swk)
{
    int result=0;
    bool done = false;
    uint32_t req_output_stream_cnt = 0;
    struct timeval tv;
    unsigned long FrameDecStartTimeUsec;
    unsigned long timeUs;

    Q_UNUSED(swk);

    if (false == m_conversionLibInited)
    {
        DBG_ERROR("ConversionLib Init Fail \n");
        return -1;
    }

    if (NULL_POINTER == p_misc_device)
    {
        DBG_ERROR("p_misc_device is NULL");
        return -1;
    }

    if ((WK_DTOF_PCM != swk) && (true == Utils::is_env_var_true(ENV_VAR_FRAME_DROP_CHECK_ENABLE)))
    {
        int lost = check_frame_loss(&flc, frm_rawdata, frm_rawdata_size);
        if (lost > 0) {
            DBG_ERROR("lost %d frames, last_id: %d, frm_sequence: %d\n", lost, flc.last_id, frm_sequence);
        }
    }

    if (false == skip_frame_decode)
    {
        depthOutputs[0].format                    = WRAPPER_CAM_FORMAT_DEPTH16;
        depthOutputs[0].formatParams.bitsPerPixel = 16;
        depthOutputs[0].formatParams.strideBytes  = m_sns_param.out_frm_width;
        depthOutputs[0].formatParams.sliceHeight  = m_sns_param.out_frm_height;
        depthOutputs[0].out_image_length          = m_sns_param.out_frm_width*m_sns_param.out_frm_height*sizeof(u16);
        depthOutputs[0].out_depth_image = (uint8_t*) depth16_buffer;
        depthOutputs[0].out_pcloud_image = point_cloud_buffer;
        req_output_stream_cnt = 1;

        depthInput.in_image    = (const int8_t*)frm_rawdata;
        depthInput.formatParams.bitsPerPixel = 8;
        depthInput.formatParams.strideBytes  = m_sns_param.raw_width;
        depthInput.formatParams.sliceHeight  = m_sns_param.raw_height;
#if !defined(ENABLE_COMPATIABLE_WITH_OLD_ALGO_LIB)
        depthInput.in_image_size    = frm_rawdata_size;
        depthInput.in_sram_id    = NULL;    // just to set to NULL for normal algo lib call
#endif

#if ALGO_LIB_VERSION_CODE >= VERSION_HEX_VALUE(3, 6, 2)
        if (true == Utils::is_env_var_true(ENV_VAR_ENABLE_ALGO_LIB_DUMP_DATA))
        {
            depthInput.dump_data    = true;
            depthInput.save_path    = DEPTH_LIB_DATA_DUMP_PATH;
        }
#endif
        //DBG_INFO( "raw_width: %d raw_height: %d out_width: %d out_height: %d\n", m_sns_param.raw_width, m_sns_param.raw_height, m_sns_param.out_frm_width, m_sns_param.out_frm_height);
    
        PrepareFrameParam(&depthConfig);
    
        if (0 == m_input_frame_cnt)
        {
            DBG_INFO("sizeof(WrapperDepthOutput): %ld, sizeof(struct SpotPoint): %ld\n", sizeof(WrapperDepthOutput), sizeof(struct SpotPoint));
        }

        gettimeofday(&tv,NULL_POINTER);
        FrameDecStartTimeUsec = tv.tv_sec*1000000 + tv.tv_usec;
        done = DepthMapWrapperProcessFrame(m_handlerDepthLib,
                                    depthInput,
                                    &depthConfig,
                                    req_output_stream_cnt,
                                    depthOutputs);
        m_input_frame_cnt++;

        if (true == Utils::is_env_var_true(ENV_VAR_TRACE_ALGO_LIB_DECODE_COSTTIME))
        {
            gettimeofday(&tv,NULL_POINTER);
            timeUs = tv.tv_sec*1000000 + tv.tv_usec;
            timeUs = (timeUs - FrameDecStartTimeUsec);
            if (WK_DTOF_PCM == swk)
            {
                DBG_NOTICE("FrameDecode() return %d for mipi frame: %d, cost time: %lu us, input_frm_cnt: %d, output_frm_cnt: %d\n",
                    done, frm_sequence, timeUs, m_input_frame_cnt, m_output_frame_cnt);
            }
            else {
                DBG_NOTICE("FrameDecode() return %d for mipi frame: %d [sram_id: %d, zone_id: %d, frame_id: %d], cost time: %lu us, work_mode: %d, input_frm_cnt: %d, output_frm_cnt: %d\n",
                    done, frm_sequence, frm_rawdata[0], frm_rawdata[1], frm_rawdata[2], timeUs, swk, m_input_frame_cnt, m_output_frame_cnt);
            }
        }
    }
    else {
        m_input_frame_cnt++;

        if (WK_DTOF_PCM != swk)
        {
            
            if(m_input_frame_cnt % sub_frame_cnt_per_image_frame == 0)
                done = 1;
            else
                done = 0;
        }
        else {
            done = 1;
        }
    }

    if (true == done)
    {
        m_output_frame_cnt++;
        result = 0;
    }
    else {
        result = -1;
    }

    return result;
}

