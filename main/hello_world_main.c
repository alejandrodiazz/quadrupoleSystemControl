/* GPIO Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/ledc.h"
#include "esp_err.h"


// START PWM STUFF !!!!!!!!!!!!!!!!!!!!!!!!
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_4_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (8) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (2650000) // Frequency in Hertz. Set frequency at 1 kHz

static void example_ledc_init(void* arg)
{
    int frequency = (int) arg;
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = frequency,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
// END PWM STUFF !!!!!!!!!!!!!!!!!!!!!!!!


/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output (ESP32C2/ESP32H2 uses GPIO8 as the second output pin)
 * GPIO19: output (ESP32C2/ESP32H2 uses GPIO9 as the second output pin)
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Note. These are the default GPIO pins to be used in the example. You can
 * change IO pins in menuconfig.
 *
 * Test:
 * Connect GPIO18(8) with GPIO4
 * Connect GPIO19(9) with GPIO5
 * Generate pulses on GPIO18(8)/19(9), that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_OUTPUT_IO_0    22
#define GPIO_OUTPUT_IO_1    21
#define a0res1k 19
#define a1res1k 18
#define a2res1k  5
#define a0res100  17
#define a1res100  16
#define a2res100  12
#define a0res10  14
#define a1res10  27
#define a2res10  26
#define a3res10  25
#define a0res1  33
#define a1res1  32
#define a2res1  35

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<a0res1k) | (1ULL<<a1res1k) | (1ULL<<a2res1k) | (1ULL<<a0res100) | (1ULL<<a1res100) | (1ULL<<a2res100) | (1ULL<<a0res10) | (1ULL<<a1res10) | (1ULL<<a2res10) | (1ULL<<a3res10) | (1ULL<<a0res1) | (1ULL<<a1res1) | (1ULL<<a2res1) )
#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0
#define GPIO_INPUT_IO_1     CONFIG_GPIO_INPUT_1
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;


static void setResistorValue(void* arg){
    int resistance = (int) arg;
    if(resistance>7000 || resistance<1){
        printf("ERROR in RESISTANCE GIVEN\n");
        return;
    }

    uint8_t res1k = (int) (resistance / 1000);
    gpio_set_level(a0res1k, (res1k>>0) & 1);    // extract the 1's place bit
    gpio_set_level(a1res1k, (res1k>>1) & 1);    // 2's bit
    gpio_set_level(a2res1k, (res1k>>2) & 1);     // 4's bit

    resistance = resistance % 1000; 
    uint8_t res100 = (int) (resistance / 100);
    gpio_set_level(a0res100, (res100>>0) & 1);  // extract the 1's place bit
    gpio_set_level(a1res100, (res100>>1) & 1);  // 2's bit
    gpio_set_level(a2res100, (res100>>2) & 1);   // 4's bit

    resistance = resistance % 100; 
    uint8_t res10 = (int) (resistance / 10);
    gpio_set_level(a0res10, (res10>>0) & 1);    // extract the 1's place bit
    gpio_set_level(a1res10, (res10>>1) & 1);    // 2's bit
    gpio_set_level(a2res10, (res10>>2) & 1);     // 4's bit
    gpio_set_level(a3res10, (res10>>3) & 1);     // 8's bit

    resistance = resistance % 10; 
    uint8_t res1 = (uint8_t) resistance;
    gpio_set_level(a0res1, (res1>>0) & 1);      // extract the 1's place bit
    gpio_set_level(a1res1, (res1>>1) & 1);      // 2's bit
    gpio_set_level(a2res1, (res1>>2) & 1);       // 4's bit

}

void app_main(void)
{

    // START PWM STUFF !!!!!!!!!!!!!!!!!!!!!!!!
    // Set the LEDC peripheral configuration
    example_ledc_init(2650000);
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    int new_duty = LEDC_DUTY;

    vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 1000 milliseconds
    printf("Running....\n");
    new_duty = LEDC_DUTY;
    printf("duty: %d, \n", new_duty);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, new_duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    // END PWM STUFF !!!!!!!!!!!!!!!!!!!!!!!!



    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    int frequency_count = 1730000;
    setResistorValue(300);  // set resistance to 300 ohms
    printf("resistance: %d\n", 300);

    while(1) {
        gpio_set_level(GPIO_OUTPUT_IO_0, 0); // enable pin set to low so that the multiplexer is working
        gpio_set_level(GPIO_OUTPUT_IO_1, 1); // latch enable pin set high so that that changes in input happen


        example_ledc_init(frequency_count);
        // Set duty to 50%
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        printf("frequency: %d\n", frequency_count);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        frequency_count = frequency_count + 5000;
        if(frequency_count >1820000){
            frequency_count = 1730000; // reset to 400KHz
        }

        // for(int i = 30; i<399; i++){
        //     setResistorValue(i);
        //     printf("resistance: %d\n", res);
        //     vTaskDelay(200 / portTICK_PERIOD_MS);
        // }

        // uint32_t time_delay = 2000; // 2 secs
        // uint32_t res = 30;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 30;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 40;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 50;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 51;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 52;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 53;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 60;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 70;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 80;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 90;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 100;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 200;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 300;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 1000;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);
        // res = 2000;
        // setResistorValue(res);
        // printf("resistance: %d\n", res);
        // vTaskDelay(time_delay / portTICK_PERIOD_MS);

    }
}

