/*
 * PX API
 * 
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* ============================================================================
 * CONFIGURATION DEFINES - Modify these to customize your device
 * ============================================================================ */

/* Device Identification */
#define PX_ZB_MANUFACTURER_NAME         "PX"
#define PX_ZB_MODEL_IDENTIFIER          "Fl5"

/* Zigbee Network Configuration */
#define PX_ZB_INSTALLCODE_POLICY        false
#define PX_ZB_ED_AGING_TIMEOUT          ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define PX_ZB_ED_KEEP_ALIVE             3000        // milliseconds

/* Endpoint Numbers */
#define PX_ZB_FLOW_ENDPOINT             10
#define PX_ZB_VOLUME_ENDPOINT           11

/* Flow Sensor Configuration (Endpoint 10, Analog Input) */
#define PX_ZB_FLOW_DESCRIPTION          "Flow [L/min]"
#define PX_ZB_FLOW_ENGINEERING_UNITS    0x0058      // BACnet: Liters per Minute
#define PX_ZB_FLOW_RESOLUTION           0.001f      // 3 decimal places
#define PX_ZB_FLOW_MIN_VALUE            0.0f        // L/min
#define PX_ZB_FLOW_MAX_VALUE            100.0f      // L/min

/* Temperature Sensor Configuration (Endpoint 10, Temperature Measurement) */
#define PX_ZB_TEMP_MIN_VALUE            (0)         // °C
#define PX_ZB_TEMP_MAX_VALUE            (80)        // °C

/* Leak Detection Configuration (Endpoint 10, Binary Input) */
#define PX_ZB_LEAK_DESCRIPTION          "Leakage detected"
#define PX_ZB_LEAK_ACTIVE_TEXT          "Yes"
#define PX_ZB_LEAK_INACTIVE_TEXT        "No"

/* Battery Configuration (Endpoint 10, Power Config) */
#define PX_ZB_BATTERY_SIZE              ESP_ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_BUILT_IN
#define PX_ZB_BATTERY_QUANTITY          1

/* Volume Sensor Configuration (Endpoint 11, Analog Input) */
#define PX_ZB_VOLUME_DESCRIPTION        "Volume [L]"
#define PX_ZB_VOLUME_ENGINEERING_UNITS  0x0052      // BACnet: Liters
#define PX_ZB_VOLUME_RESOLUTION         0.001f      // 3 decimal places
#define PX_ZB_VOLUME_MIN_VALUE          0.0f        // L
#define PX_ZB_VOLUME_MAX_VALUE          1000000.0f  // L

/* Empty Pipe Detection Configuration (Endpoint 11, Binary Input) */
#define PX_ZB_EMPTY_PIPE_DESCRIPTION    "Empty Pipe"
#define PX_ZB_EMPTY_PIPE_ACTIVE_TEXT    "Yes"
#define PX_ZB_EMPTY_PIPE_INACTIVE_TEXT  "No"

/* ============================================================================
 * API FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize the ProjectX Zigbee stack
 * 
 * This function must be called first before any other px_zigbee functions.
 * It initializes the ESP Zigbee platform and prepares the stack for use.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_init(void);

/**
 * @brief Start the Zigbee network and register the device
 * 
 * This function registers all endpoints and clusters, starts the Zigbee stack,
 * and enters the main loop. This function blocks and should be called from a
 * dedicated task.
 * 
 * @note This function does not return under normal conditions
 */
void px_zigbee_start(void *pv);

void px_zigbee_factory_reset(void);

/**
 * @brief Send a temperature value to the coordinator
 * 
 * @param temperature_celsius Temperature value in degrees Celsius
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_temperature(float temperature_celsius);

/**
 * @brief Send a flow rate value to the coordinator
 * 
 * @param flow_lpm Flow rate in Liters per Minute
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_flow(float flow_lpm);

/**
 * @brief Send a volume value to the coordinator
 * 
 * @param volume_liters Total volume in Liters
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_volume(float volume_liters);

/**
 * @brief Send battery percentage to the coordinator
 * 
 * @param percentage Battery percentage (0-100%)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_battery(uint8_t percentage);

/**
 * @brief Send leak detection status to the coordinator
 * 
 * @param detected true if leak detected, false if no leak
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_leak_status(bool detected);

/**
 * @brief Send empty pipe detection status to the coordinator
 * 
 * @param empty true if pipe is empty, false if pipe has water
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t px_zigbee_send_empty_pipe(bool empty);

/**
 * @brief Check if the device is connected to a Zigbee network
 * 
 * @return true if connected, false otherwise
 */
bool px_zigbee_is_connected(void);

#ifdef __cplusplus
}
#endif
