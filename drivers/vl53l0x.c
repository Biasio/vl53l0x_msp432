#include "vl53l0x.h"

static uint8_t stop_variable = 0;

void interrupt_gpio_init(void){
    PORT(VL53L0X_INT_PORT)->SEL0 &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);  
    PORT(VL53L0X_INT_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    PORT(VL53L0X_INT_PORT)->DIR  &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    PORT(VL53L0X_INT_PORT)->REN  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
    
    #if VL53L0X_INT_POLARITY == 0
        PORT(VL53L0X_INT_PORT)->OUT  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
        PORT(VL53L0X_INT_PORT)->IES  |=  ONE_HOT_BIT(VL53L0X_INT_PIN);
    #else if VL53L0X_INT_POLARITY == 1
        PORT(VL53L0X_INT_PORT)->OUT  &=  ~ONE_HOT_BIT(VL53L0X_INT_PIN);
        PORT(VL53L0X_INT_PORT)->IES  &=  ~ONE_HOT_BIT(VL53L0X_INT_PIN);
    #endif
    
    PORT(VL53L0X_INT_PORT)->IFG  &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
}

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
    bool success = false;

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
    success = i2c_write(
        REG_I2C_MODE, 1, 
        (uint8_t[]){0x00}, 1);

    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1);

    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1);
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    success &= i2c_read(
        REG_INTERNAL_TUNING_1, 1, 
        &stop_variable, 1);

    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1);

    return success;
}

/**
 * Wait for strobe value to be set. This is used when we read values
 * from NVM (non volatile memory).
 */
static bool read_strobe()
{
    bool success = false;
    uint8_t strobe = 0;
    if (!i2c_write(
            REG_NVM_READ_STROBE, 1, 
            (uint8_t[]){0x00}, 1)) 
    {
        return false;
    }
    do {
        success = i2c_read(
            REG_NVM_READ_STROBE, 1, 
            &strobe, 1);
    } while (success && (strobe == 0));
    if (!success) {
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
    bool success = false;
    uint8_t tmp_data8 = 0;
    uint32_t tmp_data32 = 0;
    uint8_t buf[4] = {0};

    /* Setup to read from NVM */
    success  = i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x06}, 1);
    success &= i2c_read(
        REG_NVM_READ_STROBE, 1, 
        &tmp_data8, 1);
    success &= i2c_write(
        REG_NVM_READ_STROBE, 1, 
        (uint8_t[]){tmp_data8 | 0x04}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x07}, 1);
    success &= i2c_write(
        REG_SYSTEM_HISTOGRAM_BIN, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1);
    if (!success) return false;

    /* Get the SPAD count and type */
    success &= i2c_write(
        REG_NVM_ADDR, 1, 
        (uint8_t[]){0x6b}, 1);

    if (!success) return false;
    
    if (!read_strobe()) return false;
    
    success &= i2c_read(
        REG_NVM_READ_DATA, 1, 
        buf, 4);

    if (!success) return false;

    tmp_data32 = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];

    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;


    /* Restore after reading from NVM */
    success &=i2c_write(
        REG_SYSTEM_HISTOGRAM_BIN, 1, 
        (uint8_t[]){0x00}, 1);
    success &=i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x06}, 1);
    success &=i2c_read(
        REG_NVM_READ_STROBE, 1, 
        &tmp_data8, 1);
    success &=i2c_write(
        REG_NVM_READ_STROBE, 1, 
        (uint8_t[]){tmp_data8 & 0xfb}, 1);
    success &=i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &=i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1);
    success &=i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &=i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1);

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from this register
     * instead of reading it from the NVM. */
    if (!i2c_read(
            REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 1, 
            good_spad_map, 6)) {
        return false;
    }
    return success;
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

    bool success = i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, 
        (uint8_t[]){0x2C}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 1, 
        (uint8_t[]){SPAD_START_SELECT}, 1);
    if (!success) {
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

/**
 * Load tuning settings (same as default tuning settings provided by ST api code)
 */
static bool load_default_tuning_settings()
{
    bool success = i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_SYSTEM_RANGE_CONFIG, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        0x10, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_SYSTEM_GROUPED_PARAM_HOLD, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0xFF}, 1);
    success &= i2c_write(
        REG_OSC_CALIBRATE_VAL, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, 
        (uint8_t[]){0x2C}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 
        (uint8_t[]){0x20}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 1, 
        (uint8_t[]){0x09}, 1);
    success &= i2c_write(
        REG_HISTOGRAM_CONFIG_READOUT_CTRL, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG, 1, 
        (uint8_t[]){0x04}, 1);
    success &= i2c_write(
        REG_GLOBAL_CONFIG_VCSEL_WIDTH, 1, 
        (uint8_t[]){0x03}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x40, 1, 
        (uint8_t[]){0x83}, 1);
    success &= i2c_write(
        REG_MSRC_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x25}, 1);
    success &= i2c_write(
        REG_MSRC_CONFIG_CONTROL, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_MIN_SNR, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, 1, 
        (uint8_t[]){0x06}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, 1, 
        (uint8_t[]){0x96}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x08}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x30}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x65, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x66, 1, 
        (uint8_t[]){0xA0}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_VHV_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x32}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x14}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x49, 1, 
        (uint8_t[]){0xFF}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x4A, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN, 1, 
        (uint8_t[]){0x0A}, 1);
    success &= i2c_write(
        REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_RESULT_PEAK_SIGNAL_RATE_REF, 1, 
        (uint8_t[]){0x21}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_VHV_CONFIG_LOOPBOUND, 1, 
        (uint8_t[]){0x34}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x42, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0xFF}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x45, 1, 
        (uint8_t[]){0x26}, 1);
    success &= i2c_write(
        REG_MSRC_CONFIG_TIMEOUT_MACROP, 1, 
        (uint8_t[]){0x05}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x40, 1, 
        (uint8_t[]){0x40}, 1);
    success &= i2c_write(
        REG_SYSTEM_THRESH_LOW, 1, 
        (uint8_t[]){0x06}, 1);
    success &= i2c_write(
        REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 1, 
        (uint8_t[]){0x1A}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x43, 1, 
        (uint8_t[]){0x40}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG_START, 1, 
        (uint8_t[]){0x03}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG_END, 1, 
        (uint8_t[]){0x44}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_ALGO_PHASECAL_CONFIG, 1, 
        (uint8_t[]){0x04}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x4B, 1, 
        (uint8_t[]){0x09}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x4C, 1, 
        (uint8_t[]){0x05}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x4D, 1, 
        (uint8_t[]){0x04}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_UNKNOWN_0x45, 1, 
        (uint8_t[]){0x20}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 1, 
        (uint8_t[]){0x08}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 1, 
        (uint8_t[]){0x28}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_MIN_SNR, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, 1, 
        (uint8_t[]){0x04}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, 1, 
        (uint8_t[]){0xFE}, 1);
    success &= i2c_write(
        REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        0x0D, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_SYSTEM_SEQUENCE_CONFIG, 1, 
        (uint8_t[]){0xF8}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_PAGE_REG, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1);
    return success;
}

static bool configure_interrupt(uint8_t mode)
{
    /* Interrupt on new sample ready */
    if (!i2c_write(
            REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, 
            (uint8_t[]){mode}, 1)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
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

    if (!i2c_write(
            REG_SYSTEM_INTERRUPT_CLEAR, 1, 
            (uint8_t[]){0x01}, 1)) {
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
    bool success = false;
    do {
        success = i2c_read(
            REG_RESULT_INTERRUPT_STATUS, 1, 
            &interrupt_status, 1);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
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


/* Sets the address of the VL53L0X sensor */
static bool init_address()
{
    xshut_toggle(true);
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);

    /* Wait for approx 2ms (assuming a 48MHz clock, 3 to 5 cycles per iteration)*/
    for(volatile uint32_t j = 0; j < 9600; j++);

    if (!device_is_booted()) return false;

    return true;
}


static bool init_config()
{
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);
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



void xshut_gpio_init(void)
{
    PORT(XSHUT_PORT)->SEL0 &= ~ONE_HOT_BIT(XSHUT_PIN);
    PORT(XSHUT_PORT)->SEL1 &= ~ONE_HOT_BIT(XSHUT_PIN);
    PORT(XSHUT_PORT)->DIR |= ONE_HOT_BIT(XSHUT_PIN);
    PORT(XSHUT_PORT)->OUT &= ~ONE_HOT_BIT(XSHUT_PIN);
}


void xshut_toggle(bool state)
{
    if(state) PORT(XSHUT_PORT)->OUT |= ONE_HOT_BIT(XSHUT_PIN);
    else PORT(XSHUT_PORT)->OUT &= ~ONE_HOT_BIT(XSHUT_PIN);
}


bool vl53l0x_init()
{
    if (!init_address()) return false;
    if (!init_config()) return false;
    return true;
}

bool vl53l0x_read_range_single(uint16_t *range)
{
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);
    bool success = i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_1, 1, 
        (uint8_t[]){stop_variable}, 1);
    success &= i2c_write(
        REG_SYSRANGE_START, 1, 
        (uint8_t[]){0x01}, 1);
    success &= i2c_write(
        REG_INTERNAL_TUNING_2, 1, 
        (uint8_t[]){0x00}, 1);
    success &= i2c_write(
        REG_POWER_MANAGEMENT_GO1_POWER_FORCE , 1, 
        (uint8_t[]){0x00}, 1);
    if (!success) {
        return false;
    }

    if (!i2c_write(
            REG_SYSRANGE_START, 1, 
            (uint8_t[]){0x01}, 1)) {
        return false;
    }

    uint8_t sysrange_start = 0;
    do {
        success = i2c_read(
            REG_SYSRANGE_START, 1, 
            &sysrange_start, 1);
    } while (success && (sysrange_start & 0x01));
    if (!success) {
        return false;
    }

    uint8_t interrupt_status = 0;
    do {
        success = i2c_read(
            REG_RESULT_INTERRUPT_STATUS, 1, 
            &interrupt_status, 1);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return false;
    }

    uint8_t range_status = 0;
    if (!i2c_read(REG_RESULT_RANGE_STATUS, 1, &range_status, 1)) {
        // clear interrupt before returning
        i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1);
        return false;
    }
    if (range_status & 0x07) {
        i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1);
        return false;
    }

    uint8_t range_buf[2];
    if (!i2c_read(
            REG_RESULT_RANGE_STATUS + 10, 1, 
            range_buf, 2)) {
        i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1);
        return false;
    }
    *range = ((uint16_t)range_buf[0] << 8) | range_buf[1];

    if (!i2c_write(
            REG_SYSTEM_INTERRUPT_CLEAR, 1, 
            (uint8_t[]){0x01}, 1)) {
        return false;
    }

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    return true;
}


// Sets the VL53L0X GPIO interrupt to fire in "level low" mode, meaning the INT pin asserts whenever the measured distance fall out of the threshold window.
static bool configure_LowThresh_interrupt(void)
{
    if (!configure_interrupt(0x01)) {
        return false;
    }

    //Write the low threshold. The register is 16-bit big-endian
    uint8_t low_thresh_bytes[2] = {
        (uint8_t)(VL53L0X_LOW_THRESH >> 8),    // MSB
        (uint8_t)(VL53L0X_LOW_THRESH & 0xFF)   // LSB
    };
    if (!i2c_write(REG_SYSTEM_THRESH_LOW, 1, low_thresh_bytes, 2)) {
        return false;
    }

    // Write the high threshold
    uint8_t high_thresh_bytes[2] = {
        (uint8_t)(VL53L0X_HIGH_THRESH >> 8),
        (uint8_t)(VL53L0X_HIGH_THRESH & 0xFF)
    };
    if (!i2c_write(REG_SYSTEM_THRESH_HIGH, 1, high_thresh_bytes, 2)) {
        return false;
    }

    //Configure the INT pin polarity to active LOW.
    uint8_t gpio_hv = 0;
    if (!i2c_read(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, &gpio_hv, 1)) {
        return false;
    }
    #if VL53L0X_INT_POLARITY == 0
        gpio_hv &= ~(0x10);
    #else
        gpio_hv |= (0x10);
    #endif
    
    if (!i2c_write(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1,
                        (uint8_t[]){gpio_hv}, 1)) {
        return false;
    }

    // Clear any interrupt that may already be pending on the sensor.
    if (!i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1,
                        (uint8_t[]){0x01}, 1)) {
        return false;
    }

    return true;
}



bool vl53l0x_start_continuous(void)
{
    // Configure the threshold-based interrupt before starting ranging
    if (!configure_LowThresh_interrupt()) {
        return false;
    }
    // Ensure the sensor is idle
    if (!i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1)) {
        return false;
    }
    // Wait for stop to complete
    uint8_t val=0x00;
    do {
        if (!i2c_read(REG_SYSRANGE_START, 1, &val, 1)) return false;
    } while (val & 0x01);

    // Start continuous ranging
    return i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x02}, 1);
}


bool vl53l0x_stop_continuous(void)
{
    bool status = i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1);

    status &= i2c_write(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 1, (uint8_t[]){0x01}, 1);
    status &= i2c_write(REG_INTERNAL_TUNING_2, 1, (uint8_t[]){0x01}, 1);
    status &= i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x00}, 1);
    status &= i2c_write(REG_INTERNAL_TUNING_1, 1, (uint8_t[]){0x00}, 1);
    status &= i2c_write(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1);
    status &= i2c_write(REG_INTERNAL_TUNING_2, 1, (uint8_t[]){0x00}, 1);
    status &= i2c_write(REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 1, (uint8_t[]){0x00}, 1);

    return status;
}



bool vl53l0x_read_range_interrupt(uint16_t *range)
{
    // Read the status byte and validate before trusting the range result.
    uint8_t status_byte;
    if (!i2c_read(REG_RESULT_RANGE_STATUS, 1, &status_byte, 1))
        return false;

    // Check lower 3 bits
    if (status_byte & 0x07) {
        i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1);
        return false;
    }

    // Measurement is valid. Read the 2-byte range result.
    uint8_t buf[2] = {0, 0};
    if (!i2c_read(REG_RESULT_RANGE_STATUS + 0x0A, 1, buf, 2)) {
        i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1);
        return false;
    }
    
    *range = ((uint16_t)buf[0] << 8) | buf[1];

    // Normalize out-of-range sentinel values (8190/8191 means "no target")
    if (*range >= 8190) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    // Clear the interrupt last, after all reads are complete.
    return i2c_write(REG_SYSTEM_INTERRUPT_CLEAR, 1,
                          (uint8_t[]){0x01}, 1);
}

