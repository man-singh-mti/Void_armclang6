/**
 * @file xm125_regmap.h
 * @brief XM125 I2C Distance Detector register map and bitfield definitions.
 *
 * @details
 *   - Defines register addresses, bitfields, masks, and documented default values for XM125 Distance Detector.
 *   - Excludes I2C address selection, endian helpers, and transaction APIs.
 *   - All register addresses are 16-bit, MSB first; all register data are 32-bit, MSB first.
 *   - XM125 I²C access is forbidden while MCU_INT (via PCA9534 IO6) is LOW.
 *   - Only PCA9534 can control WAKE_UP (IO5) / NRESET (IO7) and observe MCU_INT (IO6).
 *   - All distances/strengths are scaled by 1000 vs RSS API.
 *   - Only RESET MODULE command is accepted if any error bits are set in Detector Status.
 *   - All polling (MCU_INT, Busy) must have a timeout (no infinite loops).
 *   - See XM125_I2C_Distance_Detector_SPEC.md and Hardware.md for protocol and hardware invariants.
 */

#ifndef XM125_REGMAP_H_
#define XM125_REGMAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @name System/status register addresses (read-only)
 *  @{
 */
/** @brief RSS version register address. */
#define XM125_REG_VERSION_ADDR (0x0000u)
/** @brief Protocol status register address. */
#define XM125_REG_PROTOCOL_STATUS_ADDR (0x0001u)
/** @brief Measure counter register address (number of measurements since reset). */
#define XM125_REG_MEASURE_COUNTER_ADDR (0x0002u)
/** @brief Detector status register address (status and error flags). */
#define XM125_REG_DETECTOR_STATUS_ADDR (0x0003u)
/** @} */

/** @name Results register addresses (read-only)
 *  @{
 */
/** @brief Distance result register address (summary of last measurement). */
#define XM125_REG_DISTANCE_RESULT_ADDR (0x0010u)
/** @brief Peak distance and strength register addresses (0..9). */
#define XM125_REG_PEAK0_DISTANCE_ADDR (0x0011u)
#define XM125_REG_PEAK1_DISTANCE_ADDR (0x0012u)
#define XM125_REG_PEAK2_DISTANCE_ADDR (0x0013u)
#define XM125_REG_PEAK3_DISTANCE_ADDR (0x0014u)
#define XM125_REG_PEAK4_DISTANCE_ADDR (0x0015u)
#define XM125_REG_PEAK5_DISTANCE_ADDR (0x0016u)
#define XM125_REG_PEAK6_DISTANCE_ADDR (0x0017u)
#define XM125_REG_PEAK7_DISTANCE_ADDR (0x0018u)
#define XM125_REG_PEAK8_DISTANCE_ADDR (0x0019u)
#define XM125_REG_PEAK9_DISTANCE_ADDR (0x001Au)
#define XM125_REG_PEAK0_STRENGTH_ADDR (0x001Bu)
#define XM125_REG_PEAK1_STRENGTH_ADDR (0x001Cu)
#define XM125_REG_PEAK2_STRENGTH_ADDR (0x001Du)
#define XM125_REG_PEAK3_STRENGTH_ADDR (0x001Eu)
#define XM125_REG_PEAK4_STRENGTH_ADDR (0x001Fu)
#define XM125_REG_PEAK5_STRENGTH_ADDR (0x0020u)
#define XM125_REG_PEAK6_STRENGTH_ADDR (0x0021u)
#define XM125_REG_PEAK7_STRENGTH_ADDR (0x0022u)
#define XM125_REG_PEAK8_STRENGTH_ADDR (0x0023u)
#define XM125_REG_PEAK9_STRENGTH_ADDR (0x0024u)
/** @} */

/** @name Configuration register addresses (read/write)
 *  @{
 */
/** @brief Start of measured interval, mm (default: 250). */
#define XM125_REG_START_ADDR (0x0040u)
/** @brief End of measured interval, mm (default: 3000). */
#define XM125_REG_END_ADDR (0x0041u)
/** @brief Max step length, 0 = auto (default: 0). */
#define XM125_REG_MAX_STEP_LENGTH_ADDR (0x0042u)
/** @brief Enable close range leakage cancellation (default: 1/true). */
#define XM125_REG_CLOSE_RANGE_LLC_ADDR (0x0043u)
/** @brief Signal quality, int, scaled by 1000 (default: 15000). */
#define XM125_REG_SIGNAL_QUALITY_ADDR (0x0044u)
/** @brief Max profile (enum, default: 5/PROFILE5). */
#define XM125_REG_MAX_PROFILE_ADDR (0x0045u)
/** @brief Threshold method (enum, default: 3/CFAR). */
#define XM125_REG_THRESHOLD_METHOD_ADDR (0x0046u)
/** @brief Peak sorting method (enum, default: 2/STRONGEST). */
#define XM125_REG_PEAK_SORTING_ADDR (0x0047u)
/** @brief Number of frames for recorded threshold (default: 100). */
#define XM125_REG_NUM_FRAMES_RECORDED_ADDR (0x0048u)
/** @brief Fixed amplitude threshold value, scaled by 1000 (default: 100000). */
#define XM125_REG_FIXED_AMP_THRESH_ADDR (0x0049u)
/** @brief Threshold sensitivity, 0..1000, scaled by 1000 (default: 500). */
#define XM125_REG_THRESH_SENS_ADDR (0x004Au)
/** @brief Reflector shape (enum, default: 1/GENERIC). */
#define XM125_REG_REFLECTOR_SHAPE_ADDR (0x004Bu)
/** @brief Fixed strength threshold value, int, scaled by 1000 (default: 0). */
#define XM125_REG_FIXED_STRENGTH_THRESH_ADDR (0x004Cu)
/** @} */

/** @name Low power / behavior register addresses (read/write)
 *  @{
 */
/** @brief Measure on wakeup enable (default: 0/false). */
#define XM125_REG_MEASURE_ON_WAKEUP_ADDR (0x0080u)
/** @} */

/** @name Command register addresses (write-only)
 *  @{
 */
/** @brief Command register address (write-only, see @ref xm125_command_t). */
#define XM125_REG_COMMAND_ADDR (0x0100u)
/** @} */

/** @name Application ID register address (read-only)
 *  @{
 */
/** @brief Application ID register address (read-only, see @ref xm125_application_id_t). */
#define XM125_REG_APPLICATION_ID_ADDR (0xFFFFu)
/** @} */

/** @name Version register bitfields
 *  @{
 */
/** @brief Major version field position and mask. */
#define XM125_VERSION_MAJOR_POS  (16u)
#define XM125_VERSION_MAJOR_MASK (0xFFFF0000u)
/** @brief Minor version field position and mask. */
#define XM125_VERSION_MINOR_POS  (8u)
#define XM125_VERSION_MINOR_MASK (0x0000FF00u)
/** @brief Patch version field position and mask. */
#define XM125_VERSION_PATCH_POS  (0u)
#define XM125_VERSION_PATCH_MASK (0x000000FFu)
/** @} */

/** @name Protocol status bitfields
 *  @{
 */
/** @brief Protocol state error bit position and mask. */
#define XM125_PROTOCOL_STATUS_PROTOCOL_STATE_ERROR_POS  (0u)
#define XM125_PROTOCOL_STATUS_PROTOCOL_STATE_ERROR_MASK (0x00000001u)
/** @brief Packet length error bit position and mask. */
#define XM125_PROTOCOL_STATUS_PACKET_LENGTH_ERROR_POS  (1u)
#define XM125_PROTOCOL_STATUS_PACKET_LENGTH_ERROR_MASK (0x00000002u)
/** @brief Address error bit position and mask. */
#define XM125_PROTOCOL_STATUS_ADDRESS_ERROR_POS  (2u)
#define XM125_PROTOCOL_STATUS_ADDRESS_ERROR_MASK (0x00000004u)
/** @brief Write failed bit position and mask. */
#define XM125_PROTOCOL_STATUS_WRITE_FAILED_POS  (3u)
#define XM125_PROTOCOL_STATUS_WRITE_FAILED_MASK (0x00000008u)
/** @brief Write to read-only register bit position and mask. */
#define XM125_PROTOCOL_STATUS_WRITE_TO_RO_POS  (4u)
#define XM125_PROTOCOL_STATUS_WRITE_TO_RO_MASK (0x00000010u)
/** @} */

/** @name Detector status OK flags
 *  @{
 */
/** @brief Detector status OK bit positions and masks. */
#define XM125_DETECTOR_STATUS_RSS_REGISTER_OK_POS     (0u)
#define XM125_DETECTOR_STATUS_RSS_REGISTER_OK_MASK    (0x00000001u)
#define XM125_DETECTOR_STATUS_CONFIG_CREATE_OK_POS    (1u)
#define XM125_DETECTOR_STATUS_CONFIG_CREATE_OK_MASK   (0x00000002u)
#define XM125_DETECTOR_STATUS_SENSOR_CREATE_OK_POS    (2u)
#define XM125_DETECTOR_STATUS_SENSOR_CREATE_OK_MASK   (0x00000004u)
#define XM125_DETECTOR_STATUS_DETECTOR_CREATE_OK_POS  (3u)
#define XM125_DETECTOR_STATUS_DETECTOR_CREATE_OK_MASK (0x00000008u)
#define XM125_DETECTOR_STATUS_DETECTOR_BUFFER_OK_POS  (4u)
#define XM125_DETECTOR_STATUS_DETECTOR_BUFFER_OK_MASK (0x00000010u)
#define XM125_DETECTOR_STATUS_SENSOR_BUFFER_OK_POS    (5u)
#define XM125_DETECTOR_STATUS_SENSOR_BUFFER_OK_MASK   (0x00000020u)
#define XM125_DETECTOR_STATUS_CALIB_BUFFER_OK_POS     (6u)
#define XM125_DETECTOR_STATUS_CALIB_BUFFER_OK_MASK    (0x00000040u)
#define XM125_DETECTOR_STATUS_CONFIG_APPLY_OK_POS     (7u)
#define XM125_DETECTOR_STATUS_CONFIG_APPLY_OK_MASK    (0x00000080u)
#define XM125_DETECTOR_STATUS_SENSOR_CAL_OK_POS       (8u)
#define XM125_DETECTOR_STATUS_SENSOR_CAL_OK_MASK      (0x00000100u)
#define XM125_DETECTOR_STATUS_DETECTOR_CAL_OK_POS     (9u)
#define XM125_DETECTOR_STATUS_DETECTOR_CAL_OK_MASK    (0x00000200u)
/** @} */

/** @name Detector status ERROR flags
 *  @{
 */
/** @brief Detector status error bit positions and masks. */
#define XM125_DETECTOR_STATUS_RSS_REGISTER_ERR_POS     (16u)
#define XM125_DETECTOR_STATUS_RSS_REGISTER_ERR_MASK    (0x00010000u)
#define XM125_DETECTOR_STATUS_CONFIG_CREATE_ERR_POS    (17u)
#define XM125_DETECTOR_STATUS_CONFIG_CREATE_ERR_MASK   (0x00020000u)
#define XM125_DETECTOR_STATUS_SENSOR_CREATE_ERR_POS    (18u)
#define XM125_DETECTOR_STATUS_SENSOR_CREATE_ERR_MASK   (0x00040000u)
#define XM125_DETECTOR_STATUS_DETECTOR_CREATE_ERR_POS  (19u)
#define XM125_DETECTOR_STATUS_DETECTOR_CREATE_ERR_MASK (0x00080000u)
#define XM125_DETECTOR_STATUS_DETECTOR_BUFFER_ERR_POS  (20u)
#define XM125_DETECTOR_STATUS_DETECTOR_BUFFER_ERR_MASK (0x00100000u)
#define XM125_DETECTOR_STATUS_SENSOR_BUFFER_ERR_POS    (21u)
#define XM125_DETECTOR_STATUS_SENSOR_BUFFER_ERR_MASK   (0x00200000u)
#define XM125_DETECTOR_STATUS_CALIB_BUFFER_ERR_POS     (22u)
#define XM125_DETECTOR_STATUS_CALIB_BUFFER_ERR_MASK    (0x00400000u)
#define XM125_DETECTOR_STATUS_CONFIG_APPLY_ERR_POS     (23u)
#define XM125_DETECTOR_STATUS_CONFIG_APPLY_ERR_MASK    (0x00800000u)
#define XM125_DETECTOR_STATUS_SENSOR_CAL_ERR_POS       (24u)
#define XM125_DETECTOR_STATUS_SENSOR_CAL_ERR_MASK      (0x01000000u)
#define XM125_DETECTOR_STATUS_DETECTOR_CAL_ERR_POS     (25u)
#define XM125_DETECTOR_STATUS_DETECTOR_CAL_ERR_MASK    (0x02000000u)
/** @} */

/** @name Detector status other flags
 *  @{
 */
/** @brief Detector error and busy flags. */
#define XM125_DETECTOR_STATUS_DETECTOR_ERROR_POS  (28u)
#define XM125_DETECTOR_STATUS_DETECTOR_ERROR_MASK (0x10000000u)
#define XM125_DETECTOR_STATUS_BUSY_POS            (31u)
#define XM125_DETECTOR_STATUS_BUSY_MASK           (0x80000000u)
/** @} */

/**
 * @brief Helper mask for host-side “configuration OK” check.
 * @details All OK bits must be set for configuration to be considered valid.
 */
#define XM125_DETECTOR_STATUS_ALL_OK_MASK                                                                                                          \
    (XM125_DETECTOR_STATUS_RSS_REGISTER_OK_MASK | XM125_DETECTOR_STATUS_CONFIG_CREATE_OK_MASK | XM125_DETECTOR_STATUS_SENSOR_CREATE_OK_MASK |      \
     XM125_DETECTOR_STATUS_DETECTOR_CREATE_OK_MASK | XM125_DETECTOR_STATUS_DETECTOR_BUFFER_OK_MASK | XM125_DETECTOR_STATUS_SENSOR_BUFFER_OK_MASK | \
     XM125_DETECTOR_STATUS_CALIB_BUFFER_OK_MASK | XM125_DETECTOR_STATUS_CONFIG_APPLY_OK_MASK | XM125_DETECTOR_STATUS_SENSOR_CAL_OK_MASK |          \
     XM125_DETECTOR_STATUS_DETECTOR_CAL_OK_MASK)

/** @name Distance result bitfields
 *  @{
 */
/** @brief Number of detected distances field position and mask. */
#define XM125_DISTANCE_RESULT_NUM_DISTANCES_POS  (0u)
#define XM125_DISTANCE_RESULT_NUM_DISTANCES_MASK (0x0000000Fu)
/** @brief Near start edge indicator field position and mask. */
#define XM125_DISTANCE_RESULT_NEAR_START_EDGE_POS  (8u)
#define XM125_DISTANCE_RESULT_NEAR_START_EDGE_MASK (0x00000100u)
/** @brief Calibration needed indicator field position and mask. */
#define XM125_DISTANCE_RESULT_CALIBRATION_NEEDED_POS  (9u)
#define XM125_DISTANCE_RESULT_CALIBRATION_NEEDED_MASK (0x00000200u)
/** @brief Measure error indicator field position and mask. */
#define XM125_DISTANCE_RESULT_MEASURE_ERROR_POS  (10u)
#define XM125_DISTANCE_RESULT_MEASURE_ERROR_MASK (0x00000400u)
/** @brief Temperature field position and mask (deg C, relative only). */
#define XM125_DISTANCE_RESULT_TEMPERATURE_C_POS  (16u)
#define XM125_DISTANCE_RESULT_TEMPERATURE_C_MASK (0xFFFF0000u)
/** @} */

/** @name Documented default values for configuration registers
 *  @{
 */
/** @brief Default values for configuration registers (see XM125 spec for units and scaling). */
#define XM125_DEFAULT_START                 (250u)
#define XM125_DEFAULT_END                   (3000u)
#define XM125_DEFAULT_MAX_STEP_LENGTH       (0u)
#define XM125_DEFAULT_CLOSE_RANGE_LLC       (1u)    /**< True */
#define XM125_DEFAULT_SIGNAL_QUALITY        (15000) /**< int, scaled by 1000 */
#define XM125_DEFAULT_MAX_PROFILE           (5u)    /**< PROFILE5 */
#define XM125_DEFAULT_THRESHOLD_METHOD      (3u)    /**< CFAR */
#define XM125_DEFAULT_PEAK_SORTING          (2u)    /**< STRONGEST */
#define XM125_DEFAULT_NUM_FRAMES_RECORDED   (100u)
#define XM125_DEFAULT_FIXED_AMP_THRESH      (100000u)
#define XM125_DEFAULT_THRESH_SENS           (500u) /**< 0..1000, scaled by 1000 */
#define XM125_DEFAULT_REFLECTOR_SHAPE       (1u)   /**< GENERIC */
#define XM125_DEFAULT_FIXED_STRENGTH_THRESH (0)    /**< int, scaled by 1000 */
#define XM125_DEFAULT_MEASURE_ON_WAKEUP     (0u)   /**< False */
/** @} */

/**
 * @brief Max profile register values.
 * @details Selects the maximum radar profile used for measurement.
 */
typedef enum
{
    XM125_MAX_PROFILE_PROFILE1 = 1, /**< Profile 1 */
    XM125_MAX_PROFILE_PROFILE2 = 2, /**< Profile 2 */
    XM125_MAX_PROFILE_PROFILE3 = 3, /**< Profile 3 */
    XM125_MAX_PROFILE_PROFILE4 = 4, /**< Profile 4 */
    XM125_MAX_PROFILE_PROFILE5 = 5, /**< Profile 5 */
} xm125_max_profile_t;

/**
 * @brief Threshold method register values.
 * @details Selects the thresholding method for peak detection.
 */
typedef enum
{
    XM125_THRESHOLD_METHOD_FIXED_AMPLITUDE = 1, /**< Fixed amplitude threshold */
    XM125_THRESHOLD_METHOD_RECORDED        = 2, /**< Recorded threshold */
    XM125_THRESHOLD_METHOD_CFAR            = 3, /**< CFAR threshold */
    XM125_THRESHOLD_METHOD_FIXED_STRENGTH  = 4, /**< Fixed strength threshold */
} xm125_threshold_method_t;

/**
 * @brief Peak sorting method register values.
 * @details Selects sorting order for detected peaks.
 */
typedef enum
{
    XM125_PEAK_SORTING_CLOSEST   = 1, /**< Sort by range, closest first */
    XM125_PEAK_SORTING_STRONGEST = 2, /**< Sort by amplitude, strongest first */
} xm125_peak_sorting_t;

/**
 * @brief Reflector shape register values.
 * @details Selects assumed reflector shape for detection.
 */
typedef enum
{
    XM125_REFLECTOR_SHAPE_GENERIC = 1, /**< Generic reflector shape */
    XM125_REFLECTOR_SHAPE_PLANAR  = 2, /**< Planar reflector shape */
} xm125_reflector_shape_t;

/**
 * @brief Command register values.
 * @details Write to command register to execute detector actions.
 */
typedef enum
{
    XM125_COMMAND_APPLY_CONFIG_AND_CALIBRATE = 1,           /**< Apply configuration and calibrate sensor/detector */
    XM125_COMMAND_MEASURE_DISTANCE           = 2,           /**< Measure distance */
    XM125_COMMAND_APPLY_CONFIGURATION        = 3,           /**< Apply configuration only */
    XM125_COMMAND_CALIBRATE                  = 4,           /**< Calibrate sensor and detector */
    XM125_COMMAND_RECALIBRATE                = 5,           /**< Re-calibrate sensor and detector */
    XM125_COMMAND_ENABLE_UART_LOGS           = 32,          /**< Enable debug UART logs */
    XM125_COMMAND_DISABLE_UART_LOGS          = 33,          /**< Disable debug UART logs */
    XM125_COMMAND_LOG_CONFIGURATION          = 34,          /**< Print detector configuration to UART */
    XM125_COMMAND_RESET_MODULE               = 1381192737u, /**< Reset module, required for new configuration */
} xm125_command_t;

/**
 * @brief Application ID register values.
 * @details Identifies the running application on the XM125.
 */
typedef enum
{
    XM125_APPLICATION_ID_DISTANCE_DETECTOR = 1, /**< Distance Detector Application */
    XM125_APPLICATION_ID_PRESENCE_DETECTOR = 2, /**< Presence Detector Application */
    XM125_APPLICATION_ID_REF_APP_BREATHING = 3, /**< Breathing Reference Application */
    XM125_APPLICATION_ID_EXAMPLE_CARGO     = 4, /**< Cargo Example Application */
} xm125_application_id_t;

#ifdef __cplusplus
}
#endif

#endif /* XM125_REGMAP_H_ */
