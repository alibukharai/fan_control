#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits ((2 ** 10)-1)
#define LEDC_DUTY               (1000) //716 Set duty to 70%. (?/LEDC_TIMER_10_BIT) * 100% = 716
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
#define GPIO_INTERRUPT_PIN      (5)

#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_2
#define ADC_ATTEN             ADC_ATTEN_DB_11 // ADC_BITWIDTH_DEFAULT //  ADC_ATTEN_DB_11

uint16_t counter = 0;
SemaphoreHandle_t counterSemaphore;
adc_oneshot_unit_handle_t adc_handle;

void adc_read_task(void *args)
{
    while (1) {
        int raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
        printf("ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL, raw);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void IRAM_ATTR intr_handler(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(counterSemaphore, &xHigherPriorityTaskWoken);
    counter++;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void countTask(void *args)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000)); // Delay for 1 second
        xSemaphoreTake(counterSemaphore, portMAX_DELAY);
        printf("Current RPM = %d\n", counter * 30);
        counter = 0;
    }
}

void adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //ADC_WIDTH_BIT_12
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,  // Set output frequency at 25 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel = {
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = LEDC_DUTY, // Set duty to 100%
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = 1ULL << GPIO_INTERRUPT_PIN,
                             .mode = GPIO_MODE_INPUT,
                             .pull_up_en = GPIO_PULLUP_ENABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void app_main(void)
{
    adc_init();
    example_ledc_init();

    counterSemaphore = xSemaphoreCreateBinary();

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INTERRUPT_PIN, intr_handler, NULL);

    xTaskCreatePinnedToCore(countTask, "counter", 2048, NULL, 10, NULL, 0);
    xTaskCreate(adc_read_task, "adc_read", 2048, NULL, 10, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to keep the tasks running
    }
}