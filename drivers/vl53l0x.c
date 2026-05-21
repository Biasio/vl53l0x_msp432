#include "vl53l0x.h"

static uint8_t stop_variable = 0;

/* Check if the sensor is booted by reading the model id 
(There is no fresh_out_of_reset as on the vl6180x) */
static bool device_is_booted()
{
    uint8_t device_id = 0;
    if (!i2c_read(
            REG_IDENTIFICATION_MODEL_ID, 1, 
            &device_id, 1)) 
    {
        return false;
    }
    return (device_id == VL53L0X_EXPECTED_DEVICE_ID);
}


/* One time I2C device initialization */
static bool data_init()
{
    /* Set 3V3 mode */
    // first read the register to keep the other register's bits intact
    uint8_t vhv_config_scl_sda = 0;
    if (!i2c_read(
            REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, 1, 
            &vhv_config_scl_sda, 1)) 
    {
        return false;
    }
    vhv_config_scl_sda |= 0x01;
    if (!i2c_write(
            REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, 1, 
            (uint8_t[]){vhv_config_scl_sda}, 1)) 
    {
        return false;
    }

    /* Set I2C standard mode */
    if (! i2c_write(
        REG_I2C_MODE, 1, 
        (uint8_t[]){0x00}, 1))
    {
        return false;
    }

     if (! i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }

    if (! i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    
    if (! i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    if (! i2c_read(
        REG_INTERNAL_TUNING_1, 1, 
        &stop_variable, 1)) 
    {
        return false;
    }

    if (! i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (! i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (! i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }

    return true;
}

/**
 * Wait for strobe value to be set. This is used when we read values
 * from NVM (non volatile memory).
 */
static bool read_strobe()
{
    uint8_t strobe = 0;
    if (!i2c_write(
            REG_NVM_READ_STROBE, 1, 
            (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }

    if (!I2C_POLL_UNTIL(REG_NVM_READ_STROBE, &strobe, 
            (strobe != 0), TIMEOUT_POLL))
    {
        return false;
    }
    if (!i2c_write(
            REG_NVM_READ_STROBE, 1, 
            (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    return true;
}

/**
 * Gets the spad count, spad type och "good" spad map stored by ST in NVM at
 * their production line.
 * .
 * According to the datasheet, ST runs a calibration (without cover glass) and
 * saves a "good" SPAD map to NVM (non volatile memory). The SPAD array has two
 * types of SPADs: aperture and non-aperture. By default, all of the
 * good SPADs are enabled, but we should only enable a subset of them to get
 * an optimized signal rate. We should also only enable either only aperture
 * or only non-aperture SPADs. The number of SPADs to enable and which type
 * are also saved during the calibration step at ST factory and can be retrieved
 * from NVM.
 */
static bool get_spad_info_from_nvm(uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
    uint8_t tmp_data8 = 0;
    uint32_t tmp_data32 = 0;
    uint8_t buf[4] = {0};

    /* Setup to read from NVM */
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x06}, 1)) 
    {
        return false;
    }
    if (!i2c_read(
        REG_NVM_READ_STROBE, 1, 
        &tmp_data8, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_NVM_READ_STROBE, 1, 
        (uint8_t[]){tmp_data8 | 0x04}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x07}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSTEM_HISTOGRAM_BIN, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }

    /* Get the SPAD count and type */
    if (!i2c_write(
        REG_NVM_ADDR, 1, 
        (uint8_t[]){0x6b}, 1)) 
    {
        return false;
    }
    
    if (!read_strobe()) return false;
    
    if (!i2c_read(
        REG_NVM_READ_DATA, 1, 
        buf, 4)) 
    {
        return false;
    }

    tmp_data32 = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];

    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;


    /* Restore after reading from NVM */
    if (!i2c_write(
        REG_SYSTEM_HISTOGRAM_BIN, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x06}, 1)) 
    {
        return false;
    }
    if (!i2c_read(
        REG_NVM_READ_STROBE, 1, 
        &tmp_data8, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_NVM_READ_STROBE, 1, 
        (uint8_t[]){tmp_data8 & 0xfb}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from this register
     * instead of reading it from the NVM. */
    if (!i2c_read(
            REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 1, 
            good_spad_map, 6)) {
        return false;
    }
    return true;
}

/**
 * Sets the SPADs according to the value saved to NVM by ST during production. Assuming
 * similar conditions (e.g. no cover glass), this should give reasonable readings and we
 * can avoid running ref spad management (tedious code).
 */
static bool set_spads_from_nvm()
{
    uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t spads_enabled_count = 0;
    uint8_t spads_to_enable_count = 0;
    uint8_t spad_type = 0;
    volatile uint32_t total_val = 0;

    if (!get_spad_info_from_nvm(&spads_to_enable_count, &spad_type, good_spad_map)) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        total_val += good_spad_map[i];
    }

    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, 
        (uint8_t[]){0x2C}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 1, 
        (uint8_t[]){SPAD_START_SELECT}, 1)) 
    {
        return false;
    }

    uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

    /* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
     * The subset should only have the number of type enabled as suggested by the reading from
     * the NVM (spads_to_enable_count and spad_type). */
    for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
        for (int column = 0; column < SPAD_ROW_SIZE; column++) {
            int index = (row * SPAD_ROW_SIZE) + column;
            if (index >= SPAD_MAX_COUNT) {
                return false;
            }
            if (spads_enabled_count == spads_to_enable_count) {
                /* We are done */
                break;
            }
            if (index < offset) {
                continue;
            }
            if ((good_spad_map[row] >> column) & 0x1) {
                spad_map[row] |= (1 << column);
                spads_enabled_count++;
            }
        }
        if (spads_enabled_count == spads_to_enable_count) {
            /* To avoid looping unnecessarily when we are already done. */
            break;
        }
    }

    if (spads_enabled_count != spads_to_enable_count) {
        return false;
    }

    /* Write the new SPAD configuration */
    if (!i2c_write(
            REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 1, 
            spad_map, SPAD_MAP_ROW_COUNT)) {
        return false;
    }

    return true;
}

/*
Load tuning settings (same as default tuning settings provided by Pololu library
*/
static bool load_default_tuning_settings()
{
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSTEM_RANGE_CONFIG, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        0x10, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSTEM_GROUPED_PARAM_HOLD, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        0x24, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        0x25, 1, 
        (uint8_t[]){0xFF}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_OSC_CALIBRATE_VAL, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, 
        (uint8_t[]){0x2C}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 
        (uint8_t[]){0x20}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 
        (uint8_t[]){0x09}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_HISTOGRAM_CONFIG_READOUT_CTRL, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG, 1, 
        (uint8_t[]){0x04}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_GLOBAL_CONFIG_VCSEL_WIDTH, 1, 
        (uint8_t[]){0x03}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x40, 1, 
        (uint8_t[]){0x83}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_MSRC_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x25}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_MSRC_CONFIG_CONTROL, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_MIN_SNR, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, 1, 
        (uint8_t[]){0x06}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, 1, 
        (uint8_t[]){0x96}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x08}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x30}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x65, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x66, 1, 
        (uint8_t[]){0xA0}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_VHV_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x32}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x14}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x49, 1, 
        (uint8_t[]){0xFF}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x4A, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN, 1, 
        (uint8_t[]){0x0A}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_RESULT_PEAK_SIGNAL_RATE_REF, 1, 
        (uint8_t[]){0x21}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_VHV_CONFIG_LOOPBOUND, 1, 
        (uint8_t[]){0x34}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x42, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0xFF}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x45, 1, 
        (uint8_t[]){0x26}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_MSRC_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x05}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x40, 1, 
        (uint8_t[]){0x40}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSTEM_THRESH_LOW, 1, 
        (uint8_t[]){0x06}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 1, 
        (uint8_t[]){0x1A}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x43, 1, 
        (uint8_t[]){0x40}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG_START, 1, 
        (uint8_t[]){0x03}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG_END, 1, 
        (uint8_t[]){0x44}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_ALGO_PHASECAL_CONFIG, 1, 
        (uint8_t[]){0x04}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x4B, 1, 
        (uint8_t[]){0x09}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x4C, 1, 
        (uint8_t[]){0x05}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x4D, 1, 
        (uint8_t[]){0x04}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_UNKNOWN_0x45, 1, 
        (uint8_t[]){0x20}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x08}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x28}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_SNR, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, 1, 
        (uint8_t[]){0x04}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, 1, 
        (uint8_t[]){0xFE}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        0x0D, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_PAGE_REG, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    return true;
}


static bool configure_interrupt(uint8_t mode)
{
    /* Interrupt mode */
    if (!i2c_write(
            REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 
            (uint8_t[]){mode}, 1)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up */
    uint8_t gpio_hv_mux_active_high = 0;
    if (!i2c_read(
            REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, 
            &gpio_hv_mux_active_high, 1)) {
        return false;
    }
    gpio_hv_mux_active_high &= ~0x10;
    if (!i2c_write(
            REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, 
            (uint8_t[]){gpio_hv_mux_active_high}, 1)) {
        return false;
    }
    return true;
}


/**
 * Enable (or disable) specific steps in the sequence
 */
static bool set_sequence_steps_enabled(uint8_t sequence_step)
{
    return i2c_write(
        REG_SYSTEM_SEQUENCE_CONFIG, 1, 
        (uint8_t[]){sequence_step}, 1);
}

/**
 * Basic device initialization
 */
static bool static_init()
{
    if (!set_spads_from_nvm()) {
        return false;
    }

    if (!load_default_tuning_settings()) {
        return false;
    }

    // Set signal rate limit to 0.25 MCPS
    uint16_t limit = (uint16_t)(0.25 * (1 << 7));
    i2c_write(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 1, (uint8_t[]){limit >> 8, limit & 0xFF}, 2);

    if (!configure_interrupt(0x00)) {
        return false;
    }

    if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }

    return true;
}

static bool perform_single_ref_calibration(calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }
    if (!i2c_write(
            REG_SYSTEM_SEQUENCE_CONFIG, 1, 
            (uint8_t[]){sequence_config}, 1)) {
        return false;
    }
    if (!i2c_write(
            REG_SYSRANGE_START, 1, 
            (uint8_t[]){sysrange_start}, 1)) {
        return false;
    }
    /* Wait for interrupt */
    uint8_t interrupt_status = 0;

    if(!I2C_POLL_UNTIL(REG_RESULT_INTERRUPT_STATUS, &interrupt_status, 
            ((interrupt_status & 0x07) == 0), TIMEOUT_POLL)) 
    {
        return false;
    }

    if (!i2c_write(
            REG_SYSTEM_INTERRUPT_CLEAR, 1, 
            (uint8_t[]){0x01}, 1)) {
        return false;
    }

    if (!i2c_write(
            REG_SYSRANGE_START, 1, 
            (uint8_t[]){0x00}, 1)) {
        return false;
    }
    return true;
}

/**
 * Temperature calibration needs to be run again if the temperature changes by
 * more than 8 degrees according to the datasheet.
 */
static bool perform_ref_calibration()
{
    if (!perform_single_ref_calibration(CALIBRATION_TYPE_VHV)) {
        return false;
    }
    if (!perform_single_ref_calibration(CALIBRATION_TYPE_PHASE)) {
        return false;
    }
    /* Restore sequence steps enabled */
    if (!set_sequence_steps_enabled(RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }
    return true;
}


static bool init_config()
{
    if (!data_init()) {
        return false;
    }
    if (!static_init()) {
        return false;
    }
    if (!perform_ref_calibration()) {
        return false;
    }
    return true;
}


static const uint8_t interrupt_threshold_tuning[] = {
    // Start of Interrupt Threshold Settings (from ST API)
    0x01, 0xff, 0x00,
    0x01, 0x80, 0x01,
    0x01, 0xff, 0x01,
    0x01, 0x00, 0x00,
    0x01, 0xff, 0x01,
    0x01, 0x4f, 0x02,
    0x01, 0xFF, 0x0E,
    0x01, 0x00, 0x03,
    0x01, 0x01, 0x84,
    0x01, 0x02, 0x0A,
    0x01, 0x03, 0x03,
    0x01, 0x04, 0x08,
    0x01, 0x05, 0xC8,
    0x01, 0x06, 0x03,
    0x01, 0x07, 0x8D,
    0x01, 0x08, 0x08,
    0x01, 0x09, 0xC6,
    0x01, 0x0A, 0x01,
    0x01, 0x0B, 0x02,
    0x01, 0x0C, 0x00,
    0x01, 0x0D, 0xD5,
    0x01, 0x0E, 0x18,
    0x01, 0x0F, 0x12,
    0x01, 0x10, 0x01,
    0x01, 0x11, 0x82,
    0x01, 0x12, 0x00,
    0x01, 0x13, 0xD5,
    0x01, 0x14, 0x18,
    0x01, 0x15, 0x13,
    0x01, 0x16, 0x03,
    0x01, 0x17, 0x86,
    0x01, 0x18, 0x0A,
    0x01, 0x19, 0x09,
    0x01, 0x1A, 0x08,
    0x01, 0x1B, 0xC2,
    0x01, 0x1C, 0x03,
    0x01, 0x1D, 0x8F,
    0x01, 0x1E, 0x0A,
    0x01, 0x1F, 0x06,
    0x01, 0x20, 0x01,
    0x01, 0x21, 0x02,
    0x01, 0x22, 0x00,
    0x01, 0x23, 0xD5,
    0x01, 0x24, 0x18,
    0x01, 0x25, 0x22,
    0x01, 0x26, 0x01,
    0x01, 0x27, 0x82,
    0x01, 0x28, 0x00,
    0x01, 0x29, 0xD5,
    0x01, 0x2A, 0x18,
    0x01, 0x2B, 0x0B,
    0x01, 0x2C, 0x28,
    0x01, 0x2D, 0x78,
    0x01, 0x2E, 0x28,
    0x01, 0x2F, 0x91,
    0x01, 0x30, 0x00,
    0x01, 0x31, 0x0B,
    0x01, 0x32, 0x00,
    0x01, 0x33, 0x0B,
    0x01, 0x34, 0x00,
    0x01, 0x35, 0xA1,
    0x01, 0x36, 0x00,
    0x01, 0x37, 0xA0,
    0x01, 0x38, 0x00,
    0x01, 0x39, 0x04,
    0x01, 0x3A, 0x28,
    0x01, 0x3B, 0x30,
    0x01, 0x3C, 0x0C,
    0x01, 0x3D, 0x04,
    0x01, 0x3E, 0x0F,
    0x01, 0x3F, 0x79,
    0x01, 0x40, 0x28,
    0x01, 0x41, 0x1E,
    0x01, 0x42, 0x2F,
    0x01, 0x43, 0x87,
    0x01, 0x44, 0x00,
    0x01, 0x45, 0x0B,
    0x01, 0x46, 0x00,
    0x01, 0x47, 0x0B,
    0x01, 0x48, 0x00,
    0x01, 0x49, 0xA7,
    0x01, 0x4A, 0x00,
    0x01, 0x4B, 0xA6,
    0x01, 0x4C, 0x00,
    0x01, 0x4D, 0x04,
    0x01, 0x4E, 0x01,
    0x01, 0x4F, 0x00,
    0x01, 0x50, 0x00,
    0x01, 0x51, 0x80,
    0x01, 0x52, 0x09,
    0x01, 0x53, 0x08,
    0x01, 0x54, 0x01,
    0x01, 0x55, 0x00,
    0x01, 0x56, 0x0F,
    0x01, 0x57, 0x79,
    0x01, 0x58, 0x09,
    0x01, 0x59, 0x05,
    0x01, 0x5A, 0x00,
    0x01, 0x5B, 0x60,
    0x01, 0x5C, 0x05,
    0x01, 0x5D, 0xD1,
    0x01, 0x5E, 0x0C,
    0x01, 0x5F, 0x3C,
    0x01, 0x60, 0x00,
    0x01, 0x61, 0xD0,
    0x01, 0x62, 0x0B,
    0x01, 0x63, 0x03,
    0x01, 0x64, 0x28,
    0x01, 0x65, 0x10,
    0x01, 0x66, 0x2A,
    0x01, 0x67, 0x39,
    0x01, 0x68, 0x0B,
    0x01, 0x69, 0x02,
    0x01, 0x6A, 0x28,
    0x01, 0x6B, 0x10,
    0x01, 0x6C, 0x2A,
    0x01, 0x6D, 0x61,
    0x01, 0x6E, 0x0C,
    0x01, 0x6F, 0x00,
    0x01, 0x70, 0x0F,
    0x01, 0x71, 0x79,
    0x01, 0x72, 0x00,
    0x01, 0x73, 0x0B,
    0x01, 0x74, 0x00,
    0x01, 0x75, 0x0B,
    0x01, 0x76, 0x00,
    0x01, 0x77, 0xA1,
    0x01, 0x78, 0x00,
    0x01, 0x79, 0xA0,
    0x01, 0x7A, 0x00,
    0x01, 0x7B, 0x04,
    0x01, 0xFF, 0x04,
    0x01, 0x79, 0x1D,
    0x01, 0x7B, 0x27,
    0x01, 0x96, 0x0E,
    0x01, 0x97, 0xFE,
    0x01, 0x98, 0x03,
    0x01, 0x99, 0xEF,
    0x01, 0x9A, 0x02,
    0x01, 0x9B, 0x44,
    0x01, 0x73, 0x07,
    0x01, 0x70, 0x01,
    0x01, 0xff, 0x01,
    0x01, 0x00, 0x01,
    0x01, 0xff, 0x00,
    0x00, 0x00, 0x00
};

static bool load_interrupt_threshold_tuning(void) {
    uint8_t* p = interrupt_threshold_tuning;
    while (p[0] != 0) {  // stop when num_bytes is 0x00
        uint8_t num_bytes = p[0];
        uint8_t reg = p[1];
        uint8_t data[4] = {0};
        for (uint8_t i = 0; i < num_bytes; ++i) {
            data[i] = p[2 + i];
        }
        if (!i2c_write(reg, 1, data, num_bytes)) return false;
        p += 2 + num_bytes; // move to the next setting
    }
    return true;
}


// Sets the VL53L0X GPIO interrupt to fire in "level low" mode, meaning the INT pin asserts whenever the measured distance fall out of the threshold window.
static bool configure_LowThresh_interrupt(void)
{
    if(!device_is_booted()) goto CLEANUP; //check if device is booted

    // Disable interrupt first (set mode to off)
    if (!i2c_write(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 
                   (uint8_t[]){0x00}, 1)) {
        goto CLEANUP;
    }

    uint8_t interrupt_status = 0;
    if (!I2C_POLL_UNTIL(REG_RESULT_INTERRUPT_STATUS, &interrupt_status, 
            ((interrupt_status & 0x07) == 0), TIMEOUT_POLL))
    {
        goto CLEANUP;
    }

    //The low threshold register is in units of 2mm, so we need to divide the threshold by 2 before writing it to the register. 
    //Also the value is only stored in the first 12 bits of the register, so we need to mask it with 0x0FFF
    uint16_t fixed_thresh = (VL53L0X_LOW_THRESH>>1) & 0x0FFF; 

    //Write the low threshold. The register is 16-bit big-endian
    uint8_t low_thresh_bytes[2] = {
        (uint8_t)(fixed_thresh >> 8),    // MSB
        (uint8_t)(fixed_thresh & 0xFF)   // LSB
    };
    if (!i2c_write(REG_SYSTEM_THRESH_LOW, 1, low_thresh_bytes, 2)) {
        goto CLEANUP;
    }

    fixed_thresh = (VL53L0X_HIGH_THRESH>>1) & 0x0FFF; 
    // Write the high threshold
    uint8_t high_thresh_bytes[2] = {
        (uint8_t)(fixed_thresh >> 8),
        (uint8_t)(fixed_thresh & 0xFF)
    };
    if (!i2c_write(REG_SYSTEM_THRESH_HIGH, 1, high_thresh_bytes, 2)) {
        goto CLEANUP;
    }

    // If any threshold > 254 mm, load the special interrupt tuning settings
    if (VL53L0X_LOW_THRESH > 254U || VL53L0X_HIGH_THRESH > 254U) {
        if (!load_interrupt_threshold_tuning()) goto CLEANUP;
    }

    //Configure the INT pin polarity to active LOW.
    uint8_t gpio_hv = 0;
    if (!i2c_read(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, &gpio_hv, 1)) {
        goto CLEANUP;
    }
    #if VL53L0X_INT_POLARITY == 0
        gpio_hv &= ~(0x10);
    #else
        gpio_hv |= (0x10);
    #endif
    
    if (!i2c_write(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1,
                        (uint8_t[]){gpio_hv}, 1)) {
        goto CLEANUP;
    }

    uint8_t interrupt_mode=0x01;
    // Enable mode 0x01 (below LOW threshold)
    if (!i2c_write(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 
                   &interrupt_mode, 1)) {
        goto CLEANUP;
    }

    uint8_t check;
    if (!i2c_read(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 
                    &check, 1)) {
        goto CLEANUP;
    }

    if (check != interrupt_mode) goto CLEANUP;

    // Clear any interrupt that may already be pending on the sensor and return.
    return clear_interrupt();


    CLEANUP:
        clear_interrupt();
        return false;
}



void interrupt_gpio_init(void){
    PORT(VL53L0X_INT_PORT)->SEL0 &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);  
    PORT(VL53L0X_INT_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    PORT(VL53L0X_INT_PORT)->DIR  &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    PORT(VL53L0X_INT_PORT)->REN  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
    
    #if VL53L0X_INT_POLARITY == 0
        PORT(VL53L0X_INT_PORT)->OUT  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
        PORT(VL53L0X_INT_PORT)->IES  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
    #elif VL53L0X_INT_POLARITY == 1
        PORT(VL53L0X_INT_PORT)->OUT  &=  ~ONE_HOT_BIT(VL53L0X_INT_PIN);
        PORT(VL53L0X_INT_PORT)->IES  &=  ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    #endif
    
    PORT(VL53L0X_INT_PORT)->IFG  &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    PORT(VL53L0X_INT_PORT)->IE  |= ONE_HOT_BIT(VL53L0X_INT_PIN);
    NVIC_ENABLE_PORT_INT(VL53L0X_INT_PORT);
}


void xshut_gpio_init(void)
{
    //GPIO mode
    PORT(XSHUT_PORT)->SEL0 &= ~ONE_HOT_BIT(XSHUT_PIN); //0
    PORT(XSHUT_PORT)->SEL1 &= ~ONE_HOT_BIT(XSHUT_PIN); //0
    
    //output and low to keep the device in standby
    PORT(XSHUT_PORT)->DIR |= ONE_HOT_BIT(XSHUT_PIN);
    PORT(XSHUT_PORT)->OUT &= ~ONE_HOT_BIT(XSHUT_PIN);
}


bool xshut_toggle(bool state)
{
    if(state) PORT(XSHUT_PORT)->OUT |= ONE_HOT_BIT(XSHUT_PIN); //ON
    else PORT(XSHUT_PORT)->OUT &= ~ONE_HOT_BIT(XSHUT_PIN); //OFF

    return WAIT_UNTIL(device_is_booted(), 25000);
}


bool vl53l0x_init()
{
    xshut_gpio_init();
    __delay_us(1000); // small delay for power stabilization
    
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);
    
    if(!xshut_toggle(true)) return false; //check if device is booted after toggling XSHUT pin

    if (!init_config()) return false; //init config and perform reference calibration

    return true;
}



bool clear_interrupt(){
    uint8_t status;
    uint16_t timeout = 5000; // (loop iterations)

    // Clear interrupts
    if(!i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1)) return false;
    if (!i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x00}, 1)) return false;

    do {
        // Read status to check if interrupts were cleared
        if (!i2c_read(REG_RESULT_INTERRUPT_STATUS, 1, &status, 1)) return false;
        if (!(--timeout)) return false;
        __delay_us(50);
    } while ((status & 0x07) != 0);

    return true;
}




bool vl53l0x_read_range_single(uint16_t *range)
{
    if(!device_is_booted()) goto CLEANUP; //check if device is booted

    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_1, 1, 
        (uint8_t[]){stop_variable}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }

    if (!i2c_write(
            REG_SYSRANGE_START, 1, 
            (uint8_t[]){0x01}, 1)) {
        goto CLEANUP;
    }

    uint8_t sysrange_start = 0;

    if (!I2C_POLL_UNTIL(REG_SYSRANGE_START, &sysrange_start, 
            (sysrange_start & 0x01), TIMEOUT_POLL)) 
    {
        goto CLEANUP;
    }

    uint8_t interrupt_status = 0;
    bool success = false;
    do {
        success = i2c_read(
            REG_RESULT_INTERRUPT_STATUS, 1, 
            &interrupt_status, 1);
    } while (success && ((interrupt_status & 0x07) == 0));

    if (!I2C_POLL_UNTIL(REG_RESULT_INTERRUPT_STATUS, &interrupt_status, 
            ((interrupt_status & 0x07) == 0), TIMEOUT_POLL)) 
    {
        goto CLEANUP;
    }

    uint8_t range_status = 0;
    if (!i2c_read(REG_RESULT_RANGE_STATUS, 1, &range_status, 1)) {
        goto CLEANUP;
    }

    if ((range_status & 0x78) != 0x58) {
        goto CLEANUP;
    }

    uint8_t range_buf[2];
    if (!i2c_read(REG_RESULT_RANGE_STATUS + 10, 1, 
            range_buf, 2)) {
        goto CLEANUP;
    }
    *range = ((uint16_t)range_buf[0] << 8) | range_buf[1];

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    return clear_interrupt();

    CLEANUP:
        clear_interrupt();
        return false;
}



bool vl53l0x_start_continuous(void)
{
    if(!device_is_booted()) goto CLEANUP; //check if device is booted

    // Configure the threshold-based interrupt before starting ranging
    if (!configure_LowThresh_interrupt()) goto CLEANUP;

    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_1, 1, 
        (uint8_t[]){stop_variable}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }
    if (!i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1)) 
    {
        goto CLEANUP;
    }
    
    uint8_t val=0x00;
    if (!i2c_read(REG_SYSRANGE_START, 1, &val, 1)) goto CLEANUP;
    if (val & 0x01)
    { //true if it was in single mode, we need to stop it first before starting continuous mode
        // put the sesnor into idle
        if (!i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1)) goto CLEANUP;
        // Wait for stop to complete
        do {
            if (!i2c_read(REG_SYSRANGE_START, 1, &val, 1)) goto CLEANUP;
        } while (val & 0x01);
    }
    
        // Start continuous ranging
    return i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x02}, 1);

    

    CLEANUP:    
        clear_interrupt(); 
        return false;
}


bool vl53l0x_stop_continuous(void)
{
    if(!device_is_booted()) goto CLEANUP; //check if device is booted

    if (!i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1)) goto CLEANUP;
    // Wait for stop to complete
    uint8_t val=0x00;
    do {
        if (!i2c_read(REG_SYSRANGE_START, 1, &val, 1)) goto CLEANUP;
    } while (val & 0x01);

    return clear_interrupt(); 


    CLEANUP:    
        clear_interrupt(); 
        return false;
}



bool vl53l0x_read_range_interrupt(uint16_t *range, uint8_t *error_code)
{
    if(!device_is_booted()) goto CLEANUP; //check if device is booted

    // Read the status byte and validate before trusting the range result.
    if (!i2c_read(REG_RESULT_RANGE_STATUS, 1, error_code, 1)) goto CLEANUP;

    // Check lower 3 bits
    if ((*error_code & 0x78) != 0x58) goto CLEANUP;

    // Measurement is valid. Read the 2-byte range result.
    uint8_t buf[2] = {0, 0};
    if (!i2c_read(REG_RESULT_RANGE_STATUS + 0x0A, 1, buf, 2)) goto CLEANUP;
    
    *range = ((uint16_t)buf[0] << 8) | buf[1];

    // Normalize out-of-range sentinel values (8190/8191 means "no target")
    if (*range >= 8190) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    // Clear the interrupt last, after all reads are complete.
    return clear_interrupt();

    CLEANUP:    
        clear_interrupt(); 
        return false;
}
