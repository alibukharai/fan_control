#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits ((2 ** 10)-1)
#define LEDC_DUTY               (716) // Set duty to 50%. (300/LEDC_TIMER_10_BIT) * 100% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
#define GPIO_INTERRUPT_PIN      (5)

#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_2//ADC_CHANNEL_6
#define DEFAULT_VREF            3300    // Default ADC reference voltage (in mV)
#define NO_OF_SAMPLES           64      // Number of samples to take for ADC averaging

int counter = 0;
SemaphoreHandle_t counterSemaphore;
esp_adc_cal_characteristics_t adc_calib;


void adc_read_task(void *args)
{
    while (1) {
        uint32_t adc_reading = 0;
        
        // Take multiple ADC samples and average the results
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw(ADC_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;

        // Convert ADC reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_calib);
        printf("ADC Channel %d: Raw value: %d\tVoltage: %dmV\n", ADC_CHANNEL, adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void countTask(void *args)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000)); // Delay for 1 second
        xSemaphoreTake(counterSemaphore, portMAX_DELAY);
        printf("Current RPM = %d\n", counter * 30);
        counter = 0;
    }
}

void adc_init(){
        // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0); //ADC_ATTEN_DB_11

    adc1_config_width(ADC_WIDTH_BIT_12);               // Configure ADC to 12-bit width
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);  // Set attenuation for the ADC channel

    // Calibrate ADC based on measured reference voltage
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_calib);
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
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

static void IRAM_ATTR intr_handler(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(counterSemaphore, &xHigherPriorityTaskWoken);
    counter++;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
