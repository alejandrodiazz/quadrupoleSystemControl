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

#include <driver/dac.h> // needed for DAC



// START PWM STUFF !!!!!!!!!!!!!!!!!!!!!!!!
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (16) // Define the output GPIO
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
#define a0res1000 19
#define a0res100 18
#define a1res100  5
#define a2res100  17
#define a0res10  16 // have to skip 4 because that has the pwm!
#define a1res10  12
#define a2res10  14
#define a3res10  27
#define a0res1  26
#define a1res1  25
#define a2res1  33
#define a3res1  32
// #define a2res1  35

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<a0res1000) | (1ULL<<a0res100) | (1ULL<<a1res100) | (1ULL<<a2res100) | (1ULL<<a0res10) | (1ULL<<a1res10) | (1ULL<<a2res10) | (1ULL<<a3res10) | (1ULL<<a0res1) | (1ULL<<a1res1)| (1ULL<<a2res1)| (1ULL<<a3res1))
#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0
#define GPIO_INPUT_IO_1     CONFIG_GPIO_INPUT_1
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void setResistorValue(void* arg){
    int resistance = (int) arg;
    if(resistance>7000 || resistance<1){
        printf("ERROR in RESISTANCE GIVEN\n");
        return;
    }

    uint8_t res1k = (uint8_t) (resistance / 1000);
    gpio_set_level(a0res1000, ((res1k>>0) & 1));    // extract the 1's place bit

    resistance = resistance % 1000; 
    uint8_t res100 = (uint8_t) (resistance / 100);
    gpio_set_level(a0res100, ((res100>>0) & 1));  // extract the 1's place bit
    gpio_set_level(a1res100, ((res100>>1) & 1));  // 2's bit
    gpio_set_level(a2res100, ((res100>>2) & 1));   // 4's bit

    resistance = resistance % 100; 
    uint8_t res10 = (uint8_t) (resistance / 10);
    gpio_set_level(a0res10, ((res10>>0) & 1));    // extract the 1's place bit
    gpio_set_level(a1res10, ((res10>>1) & 1));    // 2's bit
    gpio_set_level(a2res10, ((res10>>2) & 1));     // 4's bit
    gpio_set_level(a3res10, ((res10>>3) & 1));     // 8's bit

    resistance = resistance % 10; 
    uint8_t res1 = (uint8_t) resistance;
    gpio_set_level(a0res1, ((res1>>0) & 1));      // extract the 1's place bit
    gpio_set_level(a1res1, ((res1>>1) & 1));      // 2's bit
    gpio_set_level(a2res1, ((res1>>2) & 1));       // 4's bit
    gpio_set_level(a3res1, ((res1>>3) & 1));       // 8's bit
    // printf("resistances: %d %d %d %d \n", res1k, res100, res10, res1);

}

// notes about I2C protocol, we have 8 registers for
// The TPS55289 operates as a target device with address 
// 74h and 75h set by the MODE pin
// START I2C STUFFFFFF
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the # of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define TPS55289_SENSOR_ADDR                 0x74        /*!< Slave address of the TPS55289 sensor */

#define TPS55289_RESET_BIT                   7

//Read a sequence of bytes from a TPS55289 sensor registers
static esp_err_t TPS55289_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, TPS55289_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Write a byte to a TPS55289 sensor register
static esp_err_t TPS55289_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, TPS55289_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

// @brief i2c master initialization
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
// END I2C STUFFF



void app_main(void)
{
    // I2C test cycle
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the TPS55289 */
    TPS55289_register_read(0x00, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    // printf(data);
    TPS55289_register_read(0x01, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    // printf(data);
    TPS55289_register_read(0x07, data, 1);
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    vTaskDelay(10000 / portTICK_PERIOD_MS);


    dac_output_enable(DAC_CHANNEL_1);       // enable DAC at GPIO25
    dac_output_voltage(DAC_CHANNEL_1, 200); // 2.59V to enable TPS55289
    // could also try this
    // gpio_set_level(DAC_CHANNEL_1, 1);


    TPS55289_register_write_byte(0x06, 0b00100000); // OFF
    vTaskDelay(10 / portTICK_PERIOD_MS);
    TPS55289_register_write_byte(0x06, 0b10100000); // ON
    printf("TPS55289 OUTPUT ON\n");
    vTaskDelay(10 / portTICK_PERIOD_MS);
    

    TPS55289_register_write_byte(0x04, 0b00000011); // setting INTFB to 11b or .0564
    // TPS55289_register_write_byte(0x04, 0b00000000); // setting INTFB to 00b or .2256


    // loop for setting voltage for Buck Boost Converter
    // int v_ref;
    // float int_fb = .0564;
    // float v_out; 
    // for( v_ref = 0; v_ref<=2047; v_ref+=50){
    //     v_out = v_ref/int_fb/1000.0;
    //     int low = v_ref  &      0b11111111;
    //     int high = (v_ref>>8) & 0b00000111;
    //     printf("%d %d %d v_out = %f\n", v_ref, low, high, v_out);
    //     TPS55289_register_write_byte(0x00, low); // setting low byte of ref voltage
    //     TPS55289_register_write_byte(0x01, high); // setting high
    //     vTaskDelay(2500 / portTICK_PERIOD_MS);
    // }

    // set register 04H to 00b so that INTFB is .2256 and we can get to lower voltages
    // set register 04H to 03b so that INTFB is .0564 to get up to high voltages



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

    int frequency_count = 1360000; //1760000;
    // setResistorValue(30);  // set resistance to 300 ohms
    // printf("resistance: %d\n", 30);
    example_ledc_init(frequency_count);
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    // // little loop for testing
    // while(1){
    //     frequency_count = 2000000;
    //     example_ledc_init(frequency_count);
    //     // Set duty to 50%
    //     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    //     // Update duty to apply the new value
    //     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    //     printf("frequency: %d\n", frequency_count);
    //     vTaskDelay(10000 / portTICK_PERIOD_MS);

    // }

    // // setResistorValue(50);
    // // printf("resistance: %d\n", 50);
    // int v_ref;
    // float int_fb = .0564;
    // float v_out; 
    // v_ref = 200;
    // v_out = v_ref/int_fb/1000.0;
    // int low = v_ref  &      0b11111111;
    // int high = (v_ref>>8) & 0b00000111;
    // printf("%d %d %d v_out = %f\n", v_ref, low, high, v_out);
    // TPS55289_register_write_byte(0x00, low); // setting low byte of ref voltage
    // TPS55289_register_write_byte(0x01, high); // setting high
    // vTaskDelay(2500 / portTICK_PERIOD_MS);

    example_ledc_init(frequency_count);
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    while(1){
        int v_ref;
        float int_fb = .0564;
        float v_out; 

        v_ref = 0;
        v_out = v_ref/int_fb/1000.0;
        int low = v_ref  &      0b11111111;
        int high = (v_ref>>8) & 0b00000111;
        printf("%d %d %d v_out = %f\n", v_ref, low, high, v_out);
        printf("Wait 40 seconds for voltage to lower\n");
        TPS55289_register_write_byte(0x00, low); // setting low byte of ref voltage
        TPS55289_register_write_byte(0x01, high); // setting high
        
        vTaskDelay(40000 / portTICK_PERIOD_MS);
        for( v_ref = 0; v_ref<=400; v_ref+=50){
            v_out = v_ref/int_fb/1000.0;
            int low = v_ref  &      0b11111111;
            int high = (v_ref>>8) & 0b00000111;
            printf("%d %d %d v_out = %f\n", v_ref, low, high, v_out);
            TPS55289_register_write_byte(0x00, low); // setting low byte of ref voltage
            TPS55289_register_write_byte(0x01, high); // setting high
            vTaskDelay(2500 / portTICK_PERIOD_MS);
        }

    }



    while(1){ // just keep waiting
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        printf("frequency: %d\n", frequency_count);
    }

    while(1) {
        gpio_set_level(GPIO_OUTPUT_IO_0, 0); // enable pin set to low so that the multiplexer is working
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_1, 1); // latch enable pin set high so that that changes in input happen


        example_ledc_init(frequency_count);
        // Set duty to 50%
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        printf("frequency: %d\n", frequency_count);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        frequency_count = frequency_count + 5000;
        if(frequency_count >1500000){
            frequency_count = 1200000; // reset to frequency
        }

        
        // for(int i = 6; i<12; i++){
        //     setResistorValue(i);
        //     printf("resistance: %d\n", i);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
        // for(int i = 12; i<30; i++){
        //     setResistorValue(i);
        //     printf("resistance: %d\n", i);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }
        // for(int i = 30; i<100; i++){
        //     setResistorValue(i);
        //     printf("resistance: %d\n", i);
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    }
}

