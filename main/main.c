#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_pm.h"
#include "rom/ets_sys.h"

// PX Headers
#include "px_zigbee.h"

#define PIN_WAKEUP_FROM_BUTTON  GPIO_NUM_0
static const char *TAG = "PX_MAIN";

// Global Status & Synchronisation
SemaphoreHandle_t sem_button_event;

//Function prototypes for tasks and setup
void button_logic_task(void *pv);
void setup_wakeup_and_interrupts(void);

// --- APP MAIN ---
void app_main(void) {
    ESP_LOGI(TAG, "System starting in Light-Sleep Mode...");

    // Init synchronization
    sem_button_event = xSemaphoreCreateBinary(); // This semaphore is given by an interrupt when a button event is detected, and taken by the button_logic_task when it starts processing the event.

    // Init power management and wakeup sources and GPIO interrupts
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true // light sleep will be automatically entered when the system is idle and no tasks are running (freertos idle)
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    setup_wakeup_and_interrupts();

    // Start tasks
    xTaskCreate(button_logic_task, "btn_task", 4096, NULL, 5, NULL);
    px_zigbee_init();
    xTaskCreate(px_zigbee_start, "Zigbee_main", 4096, NULL, 15, NULL);

    ESP_LOGI(TAG, "Initialization finished. Entering Event-Loop.");
    
    // ends the main task
    vTaskDelete(NULL);
}

// --- INTERRUPT & WAKEUP SETUP ---
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_num == PIN_WAKEUP_FROM_BUTTON) { // Button Pin
        gpio_intr_disable(PIN_WAKEUP_FROM_BUTTON); // Disable further interrupts from this pin until we have processed the current one to avoid flooding
        xSemaphoreGiveFromISR(sem_button_event, NULL);
    }
}

// Sets up that the ESP32 wakes up (from light sleep and deep sleep) when the user presses the button
// and also sets up GPIO interrupts for both pins to handle events while the ESP is awake
void setup_wakeup_and_interrupts(void) {
    // Configure GPIOs as inputs (first WITHOUT interrupts to avoid triggering interrupts during setup)
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_WAKEUP_FROM_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // Configure Wakeup (wake up when button is pressed, wakeups lightsleep and deepsleep)
    uint64_t pin_mask = (1ULL << PIN_WAKEUP_FROM_BUTTON); // | (1ULL << PIN_WAKEUP_FROM_MSP);
    esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Install ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Add ISR handlers
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_WAKEUP_FROM_BUTTON, gpio_isr_handler, (void*) PIN_WAKEUP_FROM_BUTTON));

    // Enable interrupts on the pin
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_WAKEUP_FROM_BUTTON, GPIO_INTR_HIGH_LEVEL));

    ESP_LOGI(TAG, "GPIO wakeup and interrupts configured successfully");
}

// --- TASK IMPLEMENTATIONS ---

void button_logic_task(void *pv) {
    while(1) {
        if (xSemaphoreTake(sem_button_event, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            
            if (gpio_get_level(PIN_WAKEUP_FROM_BUTTON) == 1) {
                uint32_t start_tick = xTaskGetTickCount();
                
                // Poll briefly while the button is pressed (100ms intervals)
                while(gpio_get_level(PIN_WAKEUP_FROM_BUTTON) == 1) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                uint32_t duration_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;

                if (duration_ms >= 200) { //test zigbee send
                    if (px_zigbee_is_connected()) {
                        // send random values for testing
                        px_zigbee_send_flow((float)(rand() % 1000) / 10.0f); // 0.0 to 99.9 L/min
                    }
                }
            }
        }
        gpio_intr_enable(PIN_WAKEUP_FROM_BUTTON); // Re-enable interrupts from this pin after processing is done
    }
}
