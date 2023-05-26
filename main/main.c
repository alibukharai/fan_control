#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_intr_alloc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT //LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095)//(1023)// Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) //(25000)// Frequency in Hertz. Set frequency at 5 kHz
#define GPIO_INTERRUPT_PIN 5



int counter = 0;
TaskHandle_t xHandle = NULL;

static void IRAM_ATTR intr_handler(void *args){
    counter ++;
}

void countTask(void *args){
    while(1){
        vTaskDelay(1000/portTICK_PERIOD_MS);
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Current RPM = %d\n", counter *30);
        counter = 0;
    }
    vTaskDelay(NULL);
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        // .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 500,//0 // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = 1 << GPIO_INTERRUPT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void app_main(void)
{
    example_ledc_init();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INTERRUPT_PIN, intr_handler, NULL);
    printf("Powered up :D\n");
    xTaskCreate(countTask, "counter", 2000, NULL, 20, &xHandle);
}
