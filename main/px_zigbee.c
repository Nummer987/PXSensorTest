/*
 * ProjectX Zigbee Water Flow Sensor API Implementation
 */

#include "px_zigbee.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include <zcl_utility.h>
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_pm.h"

static const char *TAG = "PX_ZIGBEE";
#define MAX_STEERING_RETRIES        30    // Max 30 retries (30 seconds)
#define CONFIGURING_TIME            15000  // ms to wait after joining before allowing sleep to ensure coordinator interview completes

/* Internal state */
static bool s_connected = false;
static bool sleep_allowed = false;
static uint8_t s_steering_retry_count = 0;

/* Helper function to convert temperature to Zigbee format */
static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

/* Zigbee action handler callback */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    
    // Log every callback with its ID
    ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "║ Zigbee Action Callback");
    ESP_LOGI(TAG, "║ Callback ID: 0x%04x (%d)", callback_id, callback_id);
    ESP_LOGI(TAG, "║ Message ptr: %p", message);
    
    if (message != NULL) {
        switch (callback_id) {
        case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID: {
            esp_zb_zcl_cmd_config_report_resp_message_t *resp = (esp_zb_zcl_cmd_config_report_resp_message_t *)message;
            ESP_LOGI(TAG, "║ Type: Report Config Response");
            ESP_LOGI(TAG, "║ Endpoint: %d", resp->info.dst_endpoint);
            ESP_LOGI(TAG, "║ Cluster: 0x%04x", resp->info.cluster);
            ESP_LOGI(TAG, "║ Status: 0x%02x", resp->info.status);
            break;
        }
        case ESP_ZB_CORE_REPORT_ATTR_CB_ID: {
            esp_zb_zcl_report_attr_message_t *report = (esp_zb_zcl_report_attr_message_t *)message;
            ESP_LOGI(TAG, "║ Type: Report Attribute");
            ESP_LOGI(TAG, "║ Cluster: 0x%04x", report->cluster);
            ESP_LOGI(TAG, "║ Status: 0x%02x", report->status);
            break;
        }
        case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID: {
            esp_zb_zcl_cmd_read_attr_resp_message_t *read_resp = (esp_zb_zcl_cmd_read_attr_resp_message_t *)message;
            ESP_LOGI(TAG, "║ Type: Read Attribute Response");
            ESP_LOGI(TAG, "║ Status: 0x%02x", read_resp->info.status);
            ESP_LOGI(TAG, "║ Cluster: 0x%04x", read_resp->info.cluster);
            ESP_LOGI(TAG, "║ Src Endpoint: %d", read_resp->info.src_endpoint);
            ESP_LOGI(TAG, "║ Dst Endpoint: %d", read_resp->info.dst_endpoint);
            break;
        }
        case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID: {
            esp_zb_zcl_cmd_write_attr_resp_message_t *write_resp = (esp_zb_zcl_cmd_write_attr_resp_message_t *)message;
            ESP_LOGI(TAG, "║ Type: Write Attribute Response");
            ESP_LOGI(TAG, "║ Status: 0x%02x", write_resp->info.status);
            ESP_LOGI(TAG, "║ Cluster: 0x%04x", write_resp->info.cluster);
            break;
        }
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: {
            esp_zb_zcl_cmd_default_resp_message_t *default_resp = (esp_zb_zcl_cmd_default_resp_message_t *)message;
            ESP_LOGI(TAG, "║ Type: Default Response");
            ESP_LOGI(TAG, "║ Status: 0x%02x", default_resp->info.status);
            ESP_LOGI(TAG, "║ Cluster: 0x%04x", default_resp->info.cluster);
            break;
        }
        default:
            ESP_LOGI(TAG, "║ Type: Unknown/Other");
            break;
        }
    } else {
        ESP_LOGI(TAG, "║ Message: NULL");
    }
    
    ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════════");
    
    return ret;
}

/* Zigbee signal handler callback */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void allow_sleep_after_join(void) {

    sleep_allowed = true;
    ESP_LOGI(TAG, "Zigbee can now sleep (join complete and configuring time elapsed)");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                s_steering_retry_count = 0; // Reset retry counter
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                s_connected = true;
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            
            s_connected = true;
            // wait for a while to allow coordinator device interview to complete before allowing sleep
            esp_zb_scheduler_alarm((esp_zb_callback_t)allow_sleep_after_join, 0, CONFIGURING_TIME);
            s_steering_retry_count = 0; // Reset counter on success
            ESP_LOGI(TAG, "Manual reporting active - coordinator reporting config ignored");
        } else {
            s_steering_retry_count++;
            ESP_LOGW(TAG, "Network steering attempt %d/%d failed (status: %s)", 
                     s_steering_retry_count, MAX_STEERING_RETRIES, esp_err_to_name(err_status));
            
            if (s_steering_retry_count < MAX_STEERING_RETRIES) {
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            } else {
                ESP_LOGE(TAG, "Failed to join network after %d attempts - giving up", MAX_STEERING_RETRIES);
            }
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        if (sleep_allowed) {
            esp_zb_sleep_now();
        }
        break;
        
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/* Register Zigbee device with all endpoints and clusters */
static void register_zigbee_device(void)
{
    ESP_LOGI(TAG, "======================================================================");
    ESP_LOGI(TAG, "Creating Zigbee Device Structure");
    ESP_LOGI(TAG, "======================================================================");
    
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    // Manufacturer Information - dynamically set length prefix
    static uint8_t manufacturer_name[64];
    static uint8_t model_identifier[64];
    
    manufacturer_name[0] = strlen(PX_ZB_MANUFACTURER_NAME);
    memcpy(&manufacturer_name[1], PX_ZB_MANUFACTURER_NAME, manufacturer_name[0]);
    
    model_identifier[0] = strlen(PX_ZB_MODEL_IDENTIFIER);
    memcpy(&model_identifier[1], PX_ZB_MODEL_IDENTIFIER, model_identifier[0]);
    
    zcl_basic_manufacturer_info_t manufacturer_info = {
        .manufacturer_name = (char *)manufacturer_name,
        .model_identifier = (char *)model_identifier,
    };

    /* ========================================================================
     * ENDPOINT 10: Flow + Temperature + Leak + Battery
     * ======================================================================== */
    esp_zb_cluster_list_t *cluster_list_ep10 = esp_zb_zcl_cluster_list_create();
    
    // Basic Cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,  // Battery powered
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list_ep10, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Flow Cluster (Analog Input)
    esp_zb_analog_input_cluster_cfg_t analog_input_cfg_flow = {
        .out_of_service = false,
        .present_value = 0.0f,
        .status_flags = ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_NORMAL,
    };
    esp_zb_attribute_list_t *flow_cluster = esp_zb_analog_input_cluster_create(&analog_input_cfg_flow);
    
    uint8_t flow_description[64];
    flow_description[0] = strlen(PX_ZB_FLOW_DESCRIPTION);
    memcpy(&flow_description[1], PX_ZB_FLOW_DESCRIPTION, flow_description[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(flow_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, flow_description));
    
    float flow_resolution = PX_ZB_FLOW_RESOLUTION;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(flow_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &flow_resolution));
    
    uint16_t flow_engineering_units = PX_ZB_FLOW_ENGINEERING_UNITS;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(flow_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_U16,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &flow_engineering_units));

    float flow_min = PX_ZB_FLOW_MIN_VALUE;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(flow_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &flow_min));

    float flow_max = PX_ZB_FLOW_MAX_VALUE;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(flow_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &flow_max));
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(cluster_list_ep10, flow_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Temperature Cluster
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = zb_temperature_to_s16(20.0f),
        .min_value = zb_temperature_to_s16(PX_ZB_TEMP_MIN_VALUE),
        .max_value = zb_temperature_to_s16(PX_ZB_TEMP_MAX_VALUE),
    };
    esp_zb_attribute_list_t *temp_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list_ep10, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Leak Detection Cluster (Binary Input)
    esp_zb_binary_input_cluster_cfg_t binary_input_cfg_leak = {
        .out_of_service = false,
        .status_flags = ESP_ZB_ZCL_BINARY_INPUT_STATUS_FLAG_NORMAL,
        .present_value = false,
    };
    esp_zb_attribute_list_t *leak_cluster = esp_zb_binary_input_cluster_create(&binary_input_cfg_leak);

    uint8_t leak_description[64];
    leak_description[0] = strlen(PX_ZB_LEAK_DESCRIPTION);
    memcpy(&leak_description[1], PX_ZB_LEAK_DESCRIPTION, leak_description[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(leak_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, leak_description));

    uint8_t leak_active_text[64];
    leak_active_text[0] = strlen(PX_ZB_LEAK_ACTIVE_TEXT);
    memcpy(&leak_active_text[1], PX_ZB_LEAK_ACTIVE_TEXT, leak_active_text[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(leak_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_ACTIVE_TEXT_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, leak_active_text));

    uint8_t leak_inactive_text[64];
    leak_inactive_text[0] = strlen(PX_ZB_LEAK_INACTIVE_TEXT);
    memcpy(&leak_inactive_text[1], PX_ZB_LEAK_INACTIVE_TEXT, leak_inactive_text[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(leak_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_INACTIVE_TEXT_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, leak_inactive_text));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(cluster_list_ep10, leak_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Battery Cluster (Power Config)
    esp_zb_power_config_cluster_cfg_t power_cfg = {
        .main_voltage = 0,
        .main_freq = 0,
        .main_alarm_mask = 0,
        .main_voltage_min = 0,
        .main_voltage_max = 0,
        .main_voltage_dwell = 0,
    };
    esp_zb_attribute_list_t *power_cluster = esp_zb_power_config_cluster_create(&power_cfg);
    
    uint8_t battery_percentage = 200;
    uint8_t battery_size = PX_ZB_BATTERY_SIZE;
    uint8_t battery_quantity = PX_ZB_BATTERY_QUANTITY;
    
    esp_zb_cluster_add_attr(power_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, 
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &battery_percentage);

    esp_zb_cluster_add_attr(power_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, 
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &battery_size);

    esp_zb_cluster_add_attr(power_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                            ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, 
                            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &battery_quantity);

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(cluster_list_ep10, power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Register Endpoint 10
    esp_zb_endpoint_config_t endpoint10_config = {
        .endpoint = PX_ZB_FLOW_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list_ep10, endpoint10_config);
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, PX_ZB_FLOW_ENDPOINT, &manufacturer_info);

    /* ========================================================================
     * ENDPOINT 11: Volume + Empty Pipe
     * ======================================================================== */
    esp_zb_cluster_list_t *cluster_list_ep11 = esp_zb_zcl_cluster_list_create();
    
    // Basic Cluster
    esp_zb_basic_cluster_cfg_t basic_cfg_ep11 = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    esp_zb_attribute_list_t *basic_cluster_ep11 = esp_zb_basic_cluster_create(&basic_cfg_ep11);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list_ep11, basic_cluster_ep11, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Volume Cluster (Analog Input)
    esp_zb_analog_input_cluster_cfg_t analog_input_cfg_volume = {
        .out_of_service = false,
        .present_value = 0.0f,
        .status_flags = ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_NORMAL,
    };
    esp_zb_attribute_list_t *volume_cluster = esp_zb_analog_input_cluster_create(&analog_input_cfg_volume);
    
    uint8_t volume_description[64];
    volume_description[0] = strlen(PX_ZB_VOLUME_DESCRIPTION);
    memcpy(&volume_description[1], PX_ZB_VOLUME_DESCRIPTION, volume_description[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(volume_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, volume_description));
    
    float volume_resolution = PX_ZB_VOLUME_RESOLUTION;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(volume_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &volume_resolution));
    
    uint16_t volume_engineering_units = PX_ZB_VOLUME_ENGINEERING_UNITS;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(volume_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_U16,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &volume_engineering_units));

    float volume_min = PX_ZB_VOLUME_MIN_VALUE;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(volume_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &volume_min));

    float volume_max = PX_ZB_VOLUME_MAX_VALUE;
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(volume_cluster, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID,
                                             ESP_ZB_ZCL_ATTR_TYPE_SINGLE,
                                             ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &volume_max));
    
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(cluster_list_ep11, volume_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Empty Pipe Cluster (Binary Input)
    esp_zb_binary_input_cluster_cfg_t binary_input_cfg_empty_pipe = {
        .out_of_service = false,
        .status_flags = ESP_ZB_ZCL_BINARY_INPUT_STATUS_FLAG_NORMAL,
        .present_value = false,
    };
    esp_zb_attribute_list_t *empty_pipe_cluster = esp_zb_binary_input_cluster_create(&binary_input_cfg_empty_pipe);

    uint8_t empty_pipe_description[64];
    empty_pipe_description[0] = strlen(PX_ZB_EMPTY_PIPE_DESCRIPTION);
    memcpy(&empty_pipe_description[1], PX_ZB_EMPTY_PIPE_DESCRIPTION, empty_pipe_description[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(empty_pipe_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, empty_pipe_description));

    uint8_t empty_pipe_active_text[64];
    empty_pipe_active_text[0] = strlen(PX_ZB_EMPTY_PIPE_ACTIVE_TEXT);
    memcpy(&empty_pipe_active_text[1], PX_ZB_EMPTY_PIPE_ACTIVE_TEXT, empty_pipe_active_text[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(empty_pipe_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_ACTIVE_TEXT_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, empty_pipe_active_text));

    uint8_t empty_pipe_inactive_text[64];
    empty_pipe_inactive_text[0] = strlen(PX_ZB_EMPTY_PIPE_INACTIVE_TEXT);
    memcpy(&empty_pipe_inactive_text[1], PX_ZB_EMPTY_PIPE_INACTIVE_TEXT, empty_pipe_inactive_text[0]);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(empty_pipe_cluster, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                            ESP_ZB_ZCL_ATTR_BINARY_INPUT_INACTIVE_TEXT_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, empty_pipe_inactive_text));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(cluster_list_ep11, empty_pipe_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Register Endpoint 11
    esp_zb_endpoint_config_t endpoint11_config = {
        .endpoint = PX_ZB_VOLUME_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list_ep11, endpoint11_config);
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, PX_ZB_VOLUME_ENDPOINT, &manufacturer_info);
    
    // Register complete device
    esp_zb_device_register(ep_list);
    
    ESP_LOGI(TAG, "======================================================================");
    ESP_LOGI(TAG, "Device Registration Complete!");
    ESP_LOGI(TAG, "======================================================================");
}

/* ============================================================================
 * PUBLIC API IMPLEMENTATION
 * ============================================================================ */

esp_err_t px_zigbee_init(void)
{
    ESP_LOGI(TAG, "Initializing PX Zigbee API");
    
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    return ESP_OK;
}

void px_zigbee_start(void *pv)
{
    ESP_LOGI(TAG, "Starting Zigbee stack");

    ESP_LOGI(TAG, "Zigbee sleep enabled");
    //esp_zb_sleep_set_threshold(500); // Sleep when idle for 20 milliseconds (default)
    esp_zb_sleep_enable(true);
    //esp_zb_set_rx_on_when_idle(false); // Keep receiver on when idle to allow incoming messages to wake up the device
    
    /* Configure Zigbee network as End Device */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = PX_ZB_INSTALLCODE_POLICY,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = PX_ZB_ED_AGING_TIMEOUT,
                .keep_alive = PX_ZB_ED_KEEP_ALIVE,
            },
        },
    };
    ESP_LOGI(TAG, "Initializing Zigbee network config");
    esp_zb_init(&zb_nwk_cfg);
    ESP_LOGI(TAG, "Zigbee init done");

    /* Register device with all endpoints and clusters */
    ESP_LOGI(TAG, "Registering Zigbee device");
    register_zigbee_device();
    ESP_LOGI(TAG, "Device registration done");
    
    /* Register action handler for Zigbee events */
    ESP_LOGI(TAG, "Registering action handler");
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    /* Start Zigbee stack */
    ESP_LOGI(TAG, "Starting Zigbee stack main");
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee start done");

    /* Set battery power descriptor must be at this position! */
    ESP_LOGI(TAG, "Setting power descriptor");
    esp_zb_set_node_descriptor_power_source(false);
    esp_zb_af_node_power_desc_t power_desc = {
        .current_power_mode = ESP_ZB_AF_NODE_POWER_MODE_COME_ON_PERIODICALLY, 
        .available_power_sources = ESP_ZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source = ESP_ZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY,
        .current_power_source_level = ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT,
    };
    esp_zb_set_node_power_descriptor(power_desc);
    ESP_LOGI(TAG, "Power descriptor set");

    // Enable Zigbee sleep with threshold
    // Zigbee will automatically sleep when idle and wake up for communication
    //esp_zb_sleep_set_threshold(20); // Sleep when idle for 20 milliseconds (default)

    /* Enter main loop (blocks) */
    ESP_LOGI(TAG, "Entering Zigbee main loop");

    esp_zb_stack_main_loop();
}

void px_zigbee_factory_reset(void)
{
    ESP_LOGI(TAG, "Performing factory reset");
    esp_zb_bdb_reset_via_local_action(); // resets 
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for reset to take effect
    esp_zb_factory_reset(); // resets zb_storage partition and restarts the device
}

esp_err_t px_zigbee_send_temperature(float temperature_celsius)
{
    int16_t measured_value = zb_temperature_to_s16(temperature_celsius);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_FLOW_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                  &measured_value,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_FLOW_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Temperature: %.1f°C (status: 0x%x)", temperature_celsius, err);
    return err;
}

esp_err_t px_zigbee_send_flow(float flow_lpm)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_FLOW_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &flow_lpm,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_FLOW_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .attributeID = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Flow: %.3f L/min (status: 0x%x)", flow_lpm, err);
    return err;
}

esp_err_t px_zigbee_send_volume(float volume_liters)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_VOLUME_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
                                  &volume_liters,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_VOLUME_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        .attributeID = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Volume: %.3f L (status: 0x%x)", volume_liters, err);
    return err;
}

esp_err_t px_zigbee_send_battery(uint8_t percentage)
{
    // Convert percentage (0-100) to Zigbee format (0-200, in 0.5% units)
    uint8_t battery_value = (percentage > 100) ? 200 : (percentage * 2);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_FLOW_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                  &battery_value,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_FLOW_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        .attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Battery: %d%% (status: 0x%x)", percentage, err);
    return err;
}

esp_err_t px_zigbee_send_leak_status(bool detected)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_FLOW_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                  &detected,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_FLOW_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Leak: %s (status: 0x%x)", detected ? "DETECTED" : "None", err);
    return err;
}

esp_err_t px_zigbee_send_empty_pipe(bool empty)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(PX_ZB_VOLUME_ENDPOINT,
                                  ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                  ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                  &empty,
                                  false);
    
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = PX_ZB_VOLUME_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
        .attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
        .manuf_code = 0,
    };
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.dis_default_resp = 1;
    report_cmd.manuf_specific = 0;
    
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Empty Pipe: %s (status: 0x%x)", empty ? "EMPTY" : "Has water", err);
    return err;
}

bool px_zigbee_is_connected(void)
{
    return s_connected;
}