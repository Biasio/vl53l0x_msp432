#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>
#include <msp432.h>
#include "i2c.h"
#include "macro.h"



/* START OF USER CONFIG */
    #ifndef VL53L0X_LOW_THRESH
    #define VL53L0X_LOW_THRESH (600U)
    #endif
    #ifndef VL53L0X_HIGH_THRESH
    #define VL53L0X_HIGH_THRESH (0xFFFFU)
    #endif


    #ifndef VL53L0X_INT_POLARITY
    /* 0 = ACTIVE_LOW, 1 = ACTIVE_HIGH */
    #define VL53L0X_INT_POLARITY 0  
    #endif

/* END OF USER CONFIG */



#define VL53L0X_OUT_OF_RANGE (8190)
#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (DEFAULT_SLAVE_ADDRESS)
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


 /* GPIO and output low */
void xshut_gpio_init(void);

/* Helper function for setting XSHOUT->OUT register 
High (state=true) or 
Low (state=false) */
void xshut_toggle(bool state);

/*  */
bool vl53l0x_init();

bool vl53l0x_read_range_single(uint16_t *range);

// Configures the threshold interrupt and then starts the sensor in continuous ranging mode. In this mode the VL53L0X takes measurements autonomously and asserts its INTERRUPT pin whenever a result crosses the threshold.
bool vl53l0x_start_continuous(void);

// Halts the ranging engine and leaves the sensor idle. You must call
// this before switching back to single-shot mode, or before putting
// the sensor itself into hardware standby via XSHUT.
bool vl53l0x_stop_continuous(void);

// Reads the range result from the sensor and then clears the sensor's
// internal interrupt latch, which physically releases the INTERRUPT pin
bool vl53l0x_read_range_interrupt(uint16_t *range);


/**************** VL53L0X REGISTER ADDRESSES ****************/

/* ── Registers exposed with REG_SYSTEM_SEQUENCE_CONFIG ──── */
/* Each bit enables a step in the ranging pipeline. The sensor executes only 
enabled steps on each measurement cycle, in this order:                      
MSRC → TCC → DSS → pre-range → final-range */

#define RANGE_SEQUENCE_STEP_MSRC        (0x04) // Bit 2: Minimum Signal Rate Check; validates near-field return before pre-range. Disable for speed.
#define RANGE_SEQUENCE_STEP_TCC         (0x10) // Bit 4: Target CentreCheck; extra signal quality verification. Rarely needed, costs time.
#define RANGE_SEQUENCE_STEP_DSS         (0x28) // Bits 3+5: Dynamic SPAD Selection; auto-picks optimal SPADs per shot. Improves stability.
#define RANGE_SEQUENCE_STEP_PRE_RANGE   (0x40) // Bit 6: coarse distance estimate; seeds the final range step. Required when final range is enabled.
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80) // Bit 7: produces the actual range result. Must always be set; disable others to trade quality for speed.

/* ── Register map ───────────────────────────── */

#define REG_SYSRANGE_START                              (0x00) // Bit 0 arms a shot (self-clearing). Mode bits: 0x00=idle, 0x01=single-shot, 0x02=back-to-back, 0x04=timed. VHV cal: write 0x41, phase cal: write 0x01. Poll bit 0 until clear to confirm start.
#define REG_SYSTEM_SEQUENCE_CONFIG                      (0x01) // Enables ranging pipeline steps via bitmask (see RANGE_SEQUENCE_STEP_* above). Default 0xE8 = final+pre+DSS. Write 0xFF to enable all steps; 0xE8 is the standard tradeoff of speed vs quality.
#define REG_SYSTEM_RANGE_CONFIG                         (0x09) // Internal ranging engine flags; written 0x00 in default tuning. Bit functions undocumented;
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                (0x0A) // Selects what asserts GPIO1: 0x00=off, 0x01=below LOW threshold, 0x02=above HIGH threshold, 0x03=out-of-window, 0x04=new sample ready. Use 0x04 for interrupt-driven acquisition.
#define REG_SYSTEM_INTERRUPT_CLEAR                      (0x0B) // Write 0x01 after reading each result to deassert GPIO1 and re-arm the interrupt. Not clearing it prevents subsequent measurements from triggering the pin.
#define REG_SYSTEM_THRESH_HIGH                          (0x0C) // 16-bit upper distance threshold in mm for GPIO interrupt modes 0x02 (above) and 0x03 (out-of-window). Write MSB first. Only meaningful when REG_SYSTEM_INTERRUPT_CONFIG_GPIO != 0x04.
#define REG_SYSTEM_THRESH_LOW                           (0x0E) // 16-bit lower distance threshold in mm for GPIO interrupt modes 0x01 (below) and 0x03 (out-of-window). Pair with THRESH_HIGH to define a detection window.
#define REG_SYSTEM_GROUPED_PARAM_HOLD                   (0x11) // UNKNOWN FUNCTION; written 0x00 during default tuning. Possibly holds parameter updates atomically across multiple registers.
#define REG_RESULT_INTERRUPT_STATUS                     (0x13) // Bits [2:0]: 0=none, 1=below LOW, 2=above HIGH, 3=out-of-window, 4=new sample. Poll until non-zero, then read result at REG_RESULT_RANGE_STATUS+10, then clear with REG_SYSTEM_INTERRUPT_CLEAR.
#define REG_RESULT_RANGE_STATUS                         (0x14) // Base of result block. Bits [7:3] at offset 0 = device error code (0x0B = no error). 2-byte range in mm (big-endian) sits at offset +10 (address 0x1E). Always check error code before trusting the range.
#define REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       (0x20) // 16-bit Q9.7 fixed-point signal rate (MCPS) subtracted from return to cancel cover-glass reflections. 0x0000 = disabled. To calibrate: point sensor at an absorber, read the peak rate, write it here.
#define REG_VHV_CONFIG_TIMEOUT_MACROP                   (0x22) // VHV calibration step timeout in macro periods (bank 1); written 0x32 in tuning. VHV sets the VCSEL reverse-bias for optimal sensitivity; longer timeout improves convergence accuracy.
#define REG_VHV_CONFIG_LOOPBOUND                        (0x23) // Maximum VHV calibration loop iteration count (bank 1); written 0x34 in tuning. Higher = more robust calibration at cost of time; lower = faster boot.
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           (0x24) // Upper VCSEL phase limit (PCLKs) for valid pre-range returns. Written 0x01 in tuning. Phase encodes time-of-flight within the measurement window; raise to accept more distant returns.
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            (0x25) // Lower VCSEL phase limit (PCLKs) for pre-range validity. Written 0xFF in tuning (= effectively no lower limit). Increase to reject near-field or zero-distance returns.
#define REG_PRE_RANGE_CONFIG_MIN_SNR                    (0x27) // Minimum SNR threshold for pre-range acceptance; written 0x00 in tuning (check disabled). Enabling this rejects weak pre-range signals at the cost of more timeouts.
#define REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM           (0x28) // Signed 8-bit mm offset added to every range result to correct per-unit mechanical variation. Loaded from factory NVM during init. Override with software after performing a manual offset calibration.
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT                (0x30) // Phase calibration timeout in macro periods. Written 0x09 in bank 0 and 0x20 in bank 1 during tuning. Longer = more accurate phase calibration; run once at startup via perform_ref_calibration().
#define REG_ALGO_PHASECAL_CONFIG                        (0x31) // Internal phase calibration step config; written 0x04 in bank 0 tuning. Bit mapping undocumented; do not change.
#define REG_GLOBAL_CONFIG_VCSEL_WIDTH                   (0x32) // VCSEL laser pulse width in clock cycles; written 0x03 in tuning. Wider pulse = more photons per shot, higher power; narrower = finer phase resolution but weaker signal.
#define REG_ALGO_PHASECAL_CONFIG_START                  (0x34) // UNKNOWN FUNCTION; written 0x03 in bank 0 tuning. Possibly defines the start phase of the calibration sweep range.
#define REG_ALGO_PHASECAL_CONFIG_END                    (0x35) // UNKNOWN FUNCTION; written 0x44 in bank 0 tuning. Possibly defines the end phase of the calibration sweep; pair with CONFIG_START.
#define REG_UNKNOWN_0x40                                (0x40) // UNKNOWN FUNCTION; written 0x83 in bank 0 and 0x40 in bank 1 during default tuning. Internal signal processing config;
#define REG_UNKNOWN_0x42                                (0x42) // UNKNOWN FUNCTION; written 0x00 in bank 1 during tuning. Bank 0 function undetermined.
#define REG_UNKNOWN_0x43                                (0x43) // UNKNOWN FUNCTION; written 0x40 in bank 1 during tuning. Bank 0 function undetermined.
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44) // High byte of 16-bit Q9.7 fixed-point minimum return signal rate (MCPS) for a valid final range. Default pair 0x00:0x20 = 0.25 MCPS. Too low accepts noise; too high rejects real measurements. Writable as a 16-bit value.
#define REG_UNKNOWN_0x45                                (0x45) // Low byte of REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT when written as a 16-bit pair. Written 0x20 in bank 0 tuning (→ 0x0020 total = 0.25 MCPS default). Written 0x26 in bank 1 (different register).
#define REG_MSRC_CONFIG_TIMEOUT_MACROP                  (0x46) // MSRC step timeout in macro periods; written 0x25 in bank 0 tuning. One macro period ≈ 2 µs at 48 MHz. Limits how long the sensor waits for a return signal before declaring MSRC failure.
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          (0x47) // Lower VCSEL phase bound (PCLKs) for valid final range returns. Written 0x08 in bank 0 tuning. Increase to reject near-field false returns in the final step.
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         (0x48) // Upper VCSEL phase bound (PCLKs) for valid final range returns. Written 0x28 (= 40 PCLKs) in bank 0 tuning. Decrease to narrow the accepted window and suppress distant multi-path echoes.
#define REG_UNKNOWN_0x49                                (0x49) // UNKNOWN FUNCTION; written 0xFF in bank 1 during tuning. Bank 0 function undetermined.
#define REG_UNKNOWN_0x4A                                (0x4A) // UNKNOWN FUNCTION; written 0x00 in bank 1 during tuning. Bank 0 function undetermined.
#define REG_UNKNOWN_0x4B                                (0x4B) // UNKNOWN FUNCTION; written 0x09 in bank 1 during tuning. Likely an internal calibration coefficient.
#define REG_UNKNOWN_0x4C                                (0x4C) // UNKNOWN FUNCTION; written 0x05 in bank 1 during tuning. Likely an internal calibration coefficient.
#define REG_UNKNOWN_0x4D                                (0x4D) // UNKNOWN FUNCTION; written 0x04 in bank 1 during tuning. Likely an internal calibration coefficient.
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         (0x4E) // Target count of reference SPADs to activate during SPAD management. Typically written 0x2C (= 44, the maximum valid count). Factory NVM value overrides this during init.
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            (0x4F) // Starting index offset into the reference SPAD enable array. Written 0x00 during init (start from index 0 of the selected quadrant, offset by aperture start index if aperture type).
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               (0x50) // Encoded VCSEL period for pre-range: actual_pclks = (reg + 1) * 2. Written 0x06 → 14 PCLKs. Longer period = wider time window, supports longer target distances but reduces resolution.
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          (0x51) // High byte of pre-range timeout encoded as (mantissa, exponent) pair with LO byte. Written 0x00:0x96 in tuning. Decode: timeout_us = ((2^exp * (mant+1)) - 1) * vcsel_macro_period_us.
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          (0x52) // Low byte of pre-range timeout (see HI byte). Written 0x96 (150) in tuning. Increase to give the sensor more time to detect pre-range returns, useful for low-reflectivity targets.
#define REG_HISTOGRAM_CONFIG_READOUT_CTRL               (0x54) // UNKNOWN FUNCTION; written 0x00 in bank 0 tuning. Likely controls histogram readout sequencing; Pololu maps the readout ctrl name to 0x55.
#define REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       (0x55) // Histogram mode readout control and initial phase selector. Only relevant in histogram measurement mode, which is not used in standard ranging operation.
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            (0x56) // Lower VCSEL phase bound (PCLKs) for valid pre-range returns. Written 0x08 in tuning. Defines the minimum expected ToF phase; raise when measuring targets closer than ~15 cm.
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           (0x57) // Upper VCSEL phase bound (PCLKs) for valid pre-range returns. Written 0x30 (= 48 PCLKs) in tuning. Must cover the full target distance range; reduce to reject farther multi-path echoes.
#define REG_MSRC_CONFIG_CONTROL                         (0x60) // Bit field disabling individual signal-rate limit checks (inverted logic: 1 = disable). Written 0x00 in tuning (all checks active). Bit 4=TCC, bit 3=MSRC, bit 1=DSS. Set bits to bypass checks for low-signal targets.
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            (0x61) // Upper sigma (standard deviation) threshold for pre-range quality, Q7 mm format. Written 0x00 in tuning (check disabled). Raise to reject high-variance, noisy pre-range estimates.
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            (0x62) // Lower sigma threshold for pre-range quality check. Written 0x00 in tuning (check disabled). Typically used together with HI to define a valid sigma band.
#define REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          (0x64) // Minimum return photon count rate (Q9.7 MCPS) to pass the pre-range signal check. Written 0x00 in tuning (disabled). Non-zero enforces a signal floor; useful to reject measurements in thick fog.
#define REG_UNKNOWN_0x65                                (0x65) // UNKNOWN FUNCTION; written 0x00 in bank 0 tuning. Internal signal processing register.
#define REG_UNKNOWN_0x66                                (0x66) // UNKNOWN FUNCTION; written 0xA0 in bank 0 tuning. Non-zero value suggests active configuration of an internal parameter.
#define REG_FINAL_RANGE_CONFIG_MIN_SNR                  (0x67) // Minimum SNR (Q7) required for the final range to be flagged valid. Written 0x00 in tuning (disabled). Enabling this rejects low-confidence results but increases the rate of ranging failures.
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             (0x70) // Encoded VCSEL period for final range: actual_pclks = (reg + 1) * 2. Written 0x04 → 10 PCLKs. Must be ≤ pre-range period. Shorter period = finer range resolution but narrower measurement window.
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        (0x71) // High byte of final range timeout (mantissa/exponent encoding, same as pre-range). Written 0x01:0xFE in tuning. Total measurement timing budget is the sum of all enabled step timeouts.
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        (0x72) // Low byte of final range timeout. Written 0xFE in tuning. Increase both HI:LO to extend the final range integration time, improving accuracy on low-reflectivity targets.
#define REG_OSC_CALIBRATE_VAL                           (0x75) // Oscillator calibration value from NVM; used to derive macro-period duration for all timeout register interpretations. Written 0x00 to clear during tuning load; restored from NVM in normal operation.
#define REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       (0x76) // UNKNOWN FUNCTION; written 0x00 in bank 0 tuning. Confirmed result registers at these names are at 0xBC+; this may be an internal ambient-noise reference counter or a write-to-clear field.
#define REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        (0x77) // UNKNOWN FUNCTION; written 0x00 in bank 0 tuning. Possibly a reference-side total photon event counter used by the internal ranging SNR algorithm. Do not use as a result register.
#define REG_RESULT_PEAK_SIGNAL_RATE_REF                 (0x78) // UNKNOWN FUNCTION; written 0x21 in bank 0 tuning. Non-zero tuning value implies it influences internal signal peak estimation.
#define REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       (0x7A) // UNKNOWN FUNCTION; written 0x0A in bank 0 tuning. Possibly the ambient-photon window count for the return path, used in background-light compensation.
#define REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        (0x7B) // UNKNOWN FUNCTION; written 0x00 in bank 0 tuning. Possibly total return-path photon event count; used internally for SNR and signal quality computation.
#define REG_POWER_MANAGEMENT_GO1_POWER_FORCE            (0x80) // Write 0x01 to force internal analog block on and unlock NVM/protected register access. Write 0x00 to release and restore auto-power management. Must bracket all NVM and stop-variable read sequences.
#define REG_SYSTEM_HISTOGRAM_BIN                        (0x81) // Doubles as NVM read-enable (write 0x01 in bank 1 after power force) and histogram bin selector in histogram mode. Not used in standard ranging; part of the NVM access preamble.
#define REG_NVM_READ_STROBE                             (0x83) // NVM access strobe. Write 0x00 to latch the address from REG_NVM_ADDR, poll until read value is non-zero (NVM word ready), then write 0x01 to release. Required for every NVM word retrieval.
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                     (0x84) // GPIO1 interrupt output polarity. Bit 4 = 1 → active-high; bit 4 = 0 → active-low. Clear bit 4 (AND ~0x10) for active-low, which is the correct setting on most breakout boards with a pull-up resistor.
#define REG_I2C_MODE                                    (0x88) // I2C bus mode selector. Write 0x00 for standard mode (100 kHz compatible). Exact bit definitions are undocumented; always write 0x00 as part of the data_init sequence.
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV           (0x89) // I/O voltage mode. Set bit 0 for 2.8 V operation; clear for 1.8 V. Must match the IOVDD supply rail. On 3.3 V MCU systems with level-shifted I2C, always set bit 0 (OR with 0x01).
#define REG_SLAVE_DEVICE_ADDRESS                        (0x8A) // 7-bit I2C slave address register. Default 0x29. Write a new 7-bit address to remap. For multi-sensor wiring: hold all but one sensor in reset via XSHUT, remap that sensor, then release the next.
#define REG_INTERNAL_TUNING_PAGE_REG                    (0x8E) // UNKNOWN FUNCTION; written 0x01 in bank 1 at the very end of the tuning load sequence, immediately before restoring bank 0. Likely a "commit" or "apply" register that latches all bank 1 tuning values.
#define REG_NVM_READ_DATA                               (0x90) // 4-byte NVM readout buffer. After completing the strobe sequence at REG_NVM_READ_STROBE, read 4 bytes here to retrieve the requested NVM word. Address 0x6B holds SPAD count/type; higher words hold calibration offsets.
#define REG_INTERNAL_TUNING_1                           (0x91) // Holds the "stop variable" — an internal ranging control token. Read once during init (while power-force is active) and cache it. Must be written back before every single ranging measurement arm sequence.
#define REG_NVM_ADDR                                    (0x94) // NVM word address to read. Write the target address (e.g., 0x6B for SPAD info), then follow the strobe protocol at REG_NVM_READ_STROBE. Each word is 4 bytes retrieved via REG_NVM_READ_DATA.
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            (0xB0) // First byte of the 6-byte (48-bit) reference SPAD enable bitmap; bits 0–43 are valid (44 max active SPADs), bits 44–47 are unused. Written with the factory-calibrated SPAD map from NVM during set_spads_from_nvm().
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT           (0xB6) // Sets the first SPAD index from which reference SPAD activation is evaluated. Written 0xB4 (= 180) to start at the aperture quadrant boundary. Changing this shifts which physical SPADs can serve as reference SPADs.
#define REG_SOFT_RESET_GO2_SOFT_RESET_N                 (0xBF) // Software reset control (active-low). Write 0x00 to assert reset; write 0x01 to release. Hardware reset via XSHUT is preferred as it guarantees a full power cycle; software reset may not clear all internal analog state.
#define REG_IDENTIFICATION_MODEL_ID                     (0xC0) // Read-only factory-programmed device model ID. Expected value: 0xEE for all VL53L0X devices. Read at boot before any configuration to verify the device is alive and communicating correctly.
#define REG_INTERNAL_TUNING_2                           (0xFF) // Bank/page selector. Write 0x01 to activate the alternate register bank, which remaps internal calibration registers onto the same address space. Write 0x00 to restore default bank. Always restore to 0x00 before normal operation.



#ifndef XSHUT_PORT
#error "XSHUT_PORT must be defined before including vl53l0x.h"
#endif

#ifndef XSHUT_PIN
#error "XSHUT_PIN must be defined before including vl53l0x.h"
#endif

#ifndef VL53L0X_INT_PORT
#error "VL53L0X_INT_PORT must be defined before including vl53l0x.h"
#endif

#ifndef VL53L0X_INT_PIN
#error "VL53L0X_INT_PIN must be defined before including vl53l0x.h"
#endif


#endif /* VL53L0X_H */
