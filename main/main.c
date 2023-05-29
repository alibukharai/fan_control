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
#include "esp_log.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 10 bits ((2 ** 10)-1)
#define LEDC_DUTY               (716) //716 Set duty to 70%. (?/LEDC_TIMER_10_BIT) * 100% = 716
#define LEDC_FREQUENCY          (25000) // Frequency in Hertz. Set frequency at 5 kHz
#define GPIO_INTERRUPT_PIN      (5)

#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_4
#define ADC_ATTEN               ADC_ATTEN_DB_11 // ADC_BITWIDTH_DEFAULT //  ADC_ATTEN_DB_11


static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

const static char *TAG = "Fan Data collection";
uint16_t counter = 0;
bool do_calibration = true;
int raw,voltage ={0,0};

SemaphoreHandle_t counterSemaphore;
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle = NULL;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

// #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
// #endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{

// #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
// #endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void adc_read_task(void *args)
{
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
          if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage));
        }
        printf("ADC%d ; Channel[%d] ; Raw Data: %d ; Cali Voltage: %d mV ;", ADC_UNIT + 1, ADC_CHANNEL, raw, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        xSemaphoreTake(counterSemaphore, portMAX_DELAY);
        printf(" RPM = %d\n", counter * 30);
        counter = 0;
    }
}

void adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //ADC_WIDTH_BIT_12
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
    do_calibration = example_adc_calibration_init(ADC_UNIT, ADC_ATTEN, &adc_cali_handle);
}

static void pwm_init(void)
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
    pwm_init();

    counterSemaphore = xSemaphoreCreateBinary();

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INTERRUPT_PIN, intr_handler, NULL);

    xTaskCreatePinnedToCore(countTask, "counter", 2048, NULL, 10, NULL, 0);
    xTaskCreate(adc_read_task, "adc_read", 2048, NULL, 10, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to keep the tasks running
    }
}