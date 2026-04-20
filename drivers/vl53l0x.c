#include "vl53l0x.h"

#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (0x29)

/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (44)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)


typedef enum {
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

static uint8_t stop_variable = 0;

void xshut_init(void)
{
    /* P1.0 GPIO and output low */
    PORT_REG(XSHUT_PORT)->SEL0 &= ~PIN_TO_BIT(XSHUT_PIN);
    PORT_REG(XSHUT_PORT)->SEL1 &= ~PIN_TO_BIT(XSHUT_PIN);
    PORT_REG(XSHUT_PORT)->DIR |= PIN_TO_BIT(XSHUT_PIN);
    PORT_REG(XSHUT_PORT)->OUT &= ~PIN_TO_BIT(XSHUT_PIN);
}

void xshut_toggle(bool state)
{
    if(state) PORT_REG(XSHUT_PORT)->OUT |= PIN_TO_BIT(XSHUT_PIN);
    else PORT_REG(XSHUT_PORT)->OUT &= ~PIN_TO_BIT(XSHUT_PIN);
}

/**
 * We can read the model id to confirm that the device is booted.
 * (There is no fresh_out_of_reset as on the vl6180x)
 */
static bool device_is_booted()
{
    uint8_t device_id = 0;
    if (!i2c_read_core(REG_IDENTIFICATION_MODEL_ID, 1, &device_id, 1)) {
        return false;
    }
    return device_id == VL53L0X_EXPECTED_DEVICE_ID;
}

/**
 * One time device initialization
 */
static bool data_init()
{
    bool success = false;

    /* Set 2v8 mode */
    uint8_t vhv_config_scl_sda = 0;
    if (!i2c_read_core(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, 1, &vhv_config_scl_sda, 1)) {
        return false;
    }
    vhv_config_scl_sda |= 0x01;
    if (!i2c_write_core(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, 1, (uint8_t[]){vhv_config_scl_sda}, 1)) {
        return false;
    }

    /* Set I2C standard mode */
    success = i2c_write_core(0x88, 1, (uint8_t[]){0x00}, 1);

    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x00}, 1);
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    success &= i2c_read_core(0x91, 1, &stop_variable, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x00}, 1);

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
    if (!i2c_write_core(0x83, 1, (uint8_t[]){0x00}, 1)) {
        return false;
    }
    do {
        success = i2c_read_core(0x83, 1, &strobe, 1);
    } while (success && (strobe == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write_core(0x83, 1, (uint8_t[]){0x01}, 1)) {
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
    success  = i2c_write_core(0x80, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x06}, 1);
    success &= i2c_read_core(0x83, 1, &tmp_data8, 1);
    success &= i2c_write_core(0x83, 1, (uint8_t[]){tmp_data8 | 0x04}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x07}, 1);
    success &= i2c_write_core(0x81, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x01}, 1);
    if (!success) {
      return false;
    }

    /* Get the SPAD count and type */
    success &= i2c_write_core(0x94, 1, (uint8_t[]){0x6b}, 1);
    if (!success) {
        return false;
    }
    if (!read_strobe()) {
        return false;
    }
    
    success &= i2c_read_core(0x90, 1, buf, 4);
    if (!success) {
        return false;
    }
    tmp_data32 = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    
    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;


    /* Restore after reading from NVM */
    success &=i2c_write_core(0x81, 1, (uint8_t[]){0x00}, 1);
    success &=i2c_write_core(0xFF, 1, (uint8_t[]){0x06}, 1);
    success &=i2c_read_core(0x83, 1, &tmp_data8, 1);
    success &=i2c_write_core(0x83, 1, (uint8_t[]){tmp_data8 & 0xfb}, 1);
    success &=i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &=i2c_write_core(0x00, 1, (uint8_t[]){0x01}, 1);
    success &=i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &=i2c_write_core(0x80, 1, (uint8_t[]){0x00}, 1);

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from this register
     * instead of reading it from the NVM. */
    if (!i2c_read_core(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 1, good_spad_map, 6)) {
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

    bool success = i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 1, (uint8_t[]){0x2C}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 1, (uint8_t[]){SPAD_START_SELECT}, 1);
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
    if (!i2c_write_core(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 1, spad_map, SPAD_MAP_ROW_COUNT)) {
        return false;
    }

    return true;
}

/**
 * Load tuning settings (same as default tuning settings provided by ST api code)
 */
static bool load_default_tuning_settings()
{
    bool success = i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x09, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x10, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x11, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x24, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x25, 1, (uint8_t[]){0xFF}, 1);
    success &= i2c_write_core(0x75, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x4E, 1, (uint8_t[]){0x2C}, 1);
    success &= i2c_write_core(0x48, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x30, 1, (uint8_t[]){0x20}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x30, 1, (uint8_t[]){0x09}, 1);
    success &= i2c_write_core(0x54, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x31, 1, (uint8_t[]){0x04}, 1);
    success &= i2c_write_core(0x32, 1, (uint8_t[]){0x03}, 1);
    success &= i2c_write_core(0x40, 1, (uint8_t[]){0x83}, 1);
    success &= i2c_write_core(0x46, 1, (uint8_t[]){0x25}, 1);
    success &= i2c_write_core(0x60, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x27, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x50, 1, (uint8_t[]){0x06}, 1);
    success &= i2c_write_core(0x51, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x52, 1, (uint8_t[]){0x96}, 1);
    success &= i2c_write_core(0x56, 1, (uint8_t[]){0x08}, 1);
    success &= i2c_write_core(0x57, 1, (uint8_t[]){0x30}, 1);
    success &= i2c_write_core(0x61, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x62, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x64, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x65, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x66, 1, (uint8_t[]){0xA0}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x22, 1, (uint8_t[]){0x32}, 1);
    success &= i2c_write_core(0x47, 1, (uint8_t[]){0x14}, 1);
    success &= i2c_write_core(0x49, 1, (uint8_t[]){0xFF}, 1);
    success &= i2c_write_core(0x4A, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x7A, 1, (uint8_t[]){0x0A}, 1);
    success &= i2c_write_core(0x7B, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x78, 1, (uint8_t[]){0x21}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x23, 1, (uint8_t[]){0x34}, 1);
    success &= i2c_write_core(0x42, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x44, 1, (uint8_t[]){0xFF}, 1);
    success &= i2c_write_core(0x45, 1, (uint8_t[]){0x26}, 1);
    success &= i2c_write_core(0x46, 1, (uint8_t[]){0x05}, 1);
    success &= i2c_write_core(0x40, 1, (uint8_t[]){0x40}, 1);
    success &= i2c_write_core(0x0E, 1, (uint8_t[]){0x06}, 1);
    success &= i2c_write_core(0x20, 1, (uint8_t[]){0x1A}, 1);
    success &= i2c_write_core(0x43, 1, (uint8_t[]){0x40}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x34, 1, (uint8_t[]){0x03}, 1);
    success &= i2c_write_core(0x35, 1, (uint8_t[]){0x44}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x31, 1, (uint8_t[]){0x04}, 1);
    success &= i2c_write_core(0x4B, 1, (uint8_t[]){0x09}, 1);
    success &= i2c_write_core(0x4C, 1, (uint8_t[]){0x05}, 1);
    success &= i2c_write_core(0x4D, 1, (uint8_t[]){0x04}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x44, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x45, 1, (uint8_t[]){0x20}, 1);
    success &= i2c_write_core(0x47, 1, (uint8_t[]){0x08}, 1);
    success &= i2c_write_core(0x48, 1, (uint8_t[]){0x28}, 1);
    success &= i2c_write_core(0x67, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x70, 1, (uint8_t[]){0x04}, 1);
    success &= i2c_write_core(0x71, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x72, 1, (uint8_t[]){0xFE}, 1);
    success &= i2c_write_core(0x76, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x77, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x0D, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x01, 1, (uint8_t[]){0xF8}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x8E, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x00}, 1);
    return success;
}

static bool configure_interrupt()
{
    /* Interrupt on new sample ready */
    if (!i2c_write_core(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 1, (uint8_t[]){0x04}, 1)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high = 0;
    if (!i2c_read_core(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, &gpio_hv_mux_active_high, 1)) {
        return false;
    }
    gpio_hv_mux_active_high &= ~0x10;
    if (!i2c_write_core(REG_GPIO_HV_MUX_ACTIVE_HIGH, 1, (uint8_t[]){gpio_hv_mux_active_high}, 1)) {
        return false;
    }

    if (!i2c_write_core(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1)) {
        return false;
    }
    return true;
}


/**
 * Enable (or disable) specific steps in the sequence
 */
static bool set_sequence_steps_enabled(uint8_t sequence_step)
{
    return i2c_write_core(REG_SYSTEM_SEQUENCE_CONFIG, 1, (uint8_t[]){sequence_step}, 1);
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

    if (!configure_interrupt()) {
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
    if (!i2c_write_core(REG_SYSTEM_SEQUENCE_CONFIG, 1, (uint8_t[]){sequence_config}, 1)) {
        return false;
    }
    if (!i2c_write_core(REG_SYSRANGE_START, 1, (uint8_t[]){sysrange_start}, 1)) {
        return false;
    }
    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    bool success = false;
    do {
        success = i2c_read_core(REG_RESULT_INTERRUPT_STATUS, 1, &interrupt_status, 1);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return false;
    }
    if (!i2c_write_core(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1)) {
        return false;
    }

    if (!i2c_write_core(REG_SYSRANGE_START, 1, (uint8_t[]){0x00}, 1)) {
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

static bool configure_address(uint8_t addr)
{
    /* 7-bit address */
    return i2c_write_core(REG_SLAVE_DEVICE_ADDRESS, 1, (uint8_t[]){addr & 0x7F}, 1);
}

/**
 * Configures the GPIOs used for the XSHUT pin.
 * Output low by default means the sensors will be in
 * hardware standby after this function is called.
 *
 * NOTE: The pins are hard-coded to P1.0, P1.1, and P1.2.
 **/
void configure_gpio()
{
    xshut_init();
}

/* Sets the address of a single VL53L0X sensor.
 * This functions assumes that all non-configured VL53L0X are still
 * in hardware standby. */
static bool init_address()
{
    xshut_toggle(true);
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);

    /* The datasheet doesn't say how long we must wait to leave hw standby,
     * but using the same delay as vl6180x seems to work fine. */
    for(volatile uint32_t j = 0; j < 400; j++); 

    if (!device_is_booted()) {
        return false;
    }

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

bool vl53l0x_init()
{
    if (!init_address()) {
        return false;
    }
    if (!init_config()) {
        return false;
    }
    return true;
}

bool vl53l0x_read_range_single(uint16_t *range)
{
    i2c_set_slave_address(VL53L0X_DEFAULT_ADDRESS);
    bool success = i2c_write_core(0x80, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x91, 1, (uint8_t[]){stop_variable}, 1);
    success &= i2c_write_core(0x00, 1, (uint8_t[]){0x01}, 1);
    success &= i2c_write_core(0xFF, 1, (uint8_t[]){0x00}, 1);
    success &= i2c_write_core(0x80, 1, (uint8_t[]){0x00}, 1);
    if (!success) {
        return false;
    }

    if (!i2c_write_core(REG_SYSRANGE_START, 1, (uint8_t[]){0x01}, 1)) {
        return false;
    }

    uint8_t sysrange_start = 0;
    do {
        success = i2c_read_core(REG_SYSRANGE_START, 1, &sysrange_start, 1);
    } while (success && (sysrange_start & 0x01));
    if (!success) {
        return false;
    }

    uint8_t interrupt_status = 0;
    do {
        success = i2c_read_core(REG_RESULT_INTERRUPT_STATUS, 1, &interrupt_status, 1);
    } while (success && ((interrupt_status & 0x07) == 0));
    if (!success) {
        return false;
    }

    uint8_t range_buf[2];
    if (!i2c_read_core(REG_RESULT_RANGE_STATUS + 10, 1, range_buf, 2)) {
        return false;
    }
    *range = ((uint16_t)range_buf[0] << 8) | range_buf[1];

    if (!i2c_write_core(REG_SYSTEM_INTERRUPT_CLEAR, 1, (uint8_t[]){0x01}, 1)) {
        return false;
    }

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    return true;
}