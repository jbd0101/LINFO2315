#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/temperature_sensor.h"

const uint8_t I2C_BMP = 0x77;
const uint8_t BMP180_START_MEASURMENT_REG = 0xF4;
const uint8_t BMP180_MEASURMENT_TEMP = 0x2E;
const uint8_t BMP180_READ_ADC_MSB_REG = 0xF6;
const uint8_t BMP180_GET_ID_REG = 0xD0;
const uint8_t BMP180_CAL_AC1_REG = 0xAA;
const uint8_t BMP180_CAL_AC2_REG = 0xAC;
const uint8_t BMP180_CAL_AC3_REG = 0xAE;
const uint8_t BMP180_CAL_AC4_REG = 0xB0;
const uint8_t BMP180_CAL_AC5_REG = 0xB2;
const uint8_t BMP180_CAL_AC6_REG = 0xB4;
const uint8_t BMP180_CAL_B1_REG = 0xB6;
const uint8_t BMP180_CAL_B2_REG = 0xB8;
const uint8_t BMP180_CAL_MB_REG = 0xBA;


const uint8_t BMP180_CAL_MC_REG = 0xBC;
const uint8_t BMP180_CAL_MD_REG = 0xBE;
const uint8_t BMP180_CHIP_ID = 0xd0;

static uint16_t bmpAC6 = 0;
static uint16_t bmpAC5 = 0;
static int16_t bmpMC = 0;
static int16_t bmpMD = 0;
static int16_t bmpAC1 = 0;
static int16_t bmpAC2 = 0;
static uint16_t bmpAC3 = 0;
static uint16_t bmpAC4 = 0;
static int16_t bmpB1 = 0;
static int16_t bmpB2 = 0;
static int16_t bmpMB = 0;

static QueueHandle_t queue;
static SemaphoreHandle_t sem1;
static SemaphoreHandle_t led;
static temperature_sensor_handle_t temp_handle;
static temperature_sensor_config_t temp_sensor={
    .range_min = 20,
    .range_max = 50,
    };
#define mode_radar 0
#define mode_bmp 1


#define RADAR_ECHO 45
#define RADAR_TRIG 46
#define LED 35
#define BUTTON_PIN 0

#define I2C_MASTER_SCL_IO    45         // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO    46         // GPIO number for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0  // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ   100000     // I2C master clock frequency
#define BMP180_ADDR          0x77       // BMP180 I2C address


/**
 * Main function called by the ESP-IDF SDK.
 */

 int32_t computeB5(uint16_t ut, uint16_t ac5, uint16_t ac6, int16_t mc, int16_t md) {
    int32_t x1 = ((int32_t) ut - (int32_t) ac6) * (int32_t) ac5 >> 15;
    int32_t x2 = ((int32_t) mc << 11) / (x1 + (int32_t) md);
    return x1 + x2;
}


void toggleLed(void *args){
    uint8_t status = 0;
    while(1){
        xSemaphoreTake(led, portMAX_DELAY);
        status = status == 0 ? 1 : 0;
        gpio_set_level(LED,status);
        ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
        // Get converted sensor data
        float tsens_out;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
        printf("Temperature in %f °C\n", tsens_out);
        // Disable the temperature sensor if it's not needed and save the power
        ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

    }
}

static void IRAM_ATTR button_isr(void *args){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(led,&xHigherPriorityTaskWoken);
}

//setup input for radar
void setup_radar(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RADAR_ECHO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RADAR_TRIG) | (1ULL << LED); 
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    
    // Attach the button ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);

    queue = xQueueCreate(100,sizeof(int64_t));
    sem1 = xSemaphoreCreateBinary();
    led = xSemaphoreCreateBinary();

}

uint8_t busy_wait(int pin, uint8_t value,uint32_t timeout){
    while(gpio_get_level(pin) != value){
        timeout --;
        if(timeout==0) return 0;
        esp_rom_delay_us(1);
    }
    return 1;
}

void radar_sensor(void *args){
    xSemaphoreGive(sem1);
    printf("rad creating data\n");

    while(1){

        gpio_set_level(RADAR_TRIG,0);
        esp_rom_delay_us(2);
        gpio_set_level(RADAR_TRIG,1);
        esp_rom_delay_us(10);
        gpio_set_level(RADAR_TRIG,0);
        uint8_t status = 1;
        if(busy_wait(RADAR_ECHO,1,1000000)==0){
            printf("error busy wait \n");
            status =0;
        }

        int64_t time = esp_timer_get_time();
        if(busy_wait(RADAR_ECHO,0,1000000)==0){
            printf("error busy wait 2 \n");
            status = 0;
        }
        time = esp_timer_get_time() - time;

        if(status==1){
            xQueueSend(queue, &time,portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void convert_radar_sensor(){
    xSemaphoreTake(sem1,portMAX_DELAY);
    printf("time convert up adn running\n");
    while(1){
        int64_t data;
        xQueueReceive(queue,&data, portMAX_DELAY);
        int distance = data*340/20000;
        printf("distance:  %d \n",distance);
    }
}
void bmp180_read8(uint8_t reg, uint8_t *value){


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,BMP180_ADDR << 1 | I2C_MASTER_WRITE,1);
    i2c_master_write_byte(cmd,reg,1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM,cmd,1000);
    i2c_cmd_link_delete(cmd);


    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,BMP180_ADDR << 1 | I2C_MASTER_READ,1);
    i2c_master_read_byte(cmd, value,I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM,cmd,1000);
    i2c_cmd_link_delete(cmd); 



}


void bmp180_read16(uint8_t reg, uint8_t *lsb,uint8_t * msb){

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd,BMP180_ADDR << 1 | I2C_MASTER_WRITE,1);
        i2c_master_write_byte(cmd,reg,1);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM,cmd,1000);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd,BMP180_ADDR << 1 | I2C_MASTER_READ,1);
        i2c_master_read_byte(cmd,msb,I2C_MASTER_ACK);
        i2c_master_read_byte(cmd,lsb,I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM,cmd,1000);
        i2c_cmd_link_delete(cmd);
}
void start_read_bmp(){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,BMP180_ADDR << 1 | I2C_MASTER_WRITE,1);
    i2c_master_write_byte(cmd,BMP180_START_MEASURMENT_REG,1);
    i2c_master_write_byte(cmd,BMP180_MEASURMENT_TEMP,1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM,cmd,1000);
    i2c_cmd_link_delete(cmd);
 }

void setup_bmp(){

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    // read chip id
    uint8_t chip_id;
    bmp180_read8(BMP180_CHIP_ID, &chip_id);
    if(chip_id != 0x55){
        printf("bad chip id, do i am connected ? ? \n");
    }else{
        printf("chip id: %d \n",chip_id);
    }

    bmp180_read16(BMP180_CAL_AC5_REG,(uint8_t *) &bmpAC5, ((uint8_t *) (&bmpAC5))+1);
    bmp180_read16(BMP180_CAL_AC6_REG,(uint8_t *) &bmpAC6, ((uint8_t *) (&bmpAC6))+1);
    bmp180_read16(BMP180_CAL_MC_REG, (uint8_t *) &bmpMC, ((uint8_t *) (&bmpMC))+1);
    bmp180_read16(BMP180_CAL_MD_REG, (uint8_t *) &bmpMD, ((uint8_t *) (&bmpMD))+1);




    printf("chip id: %d \n",chip_id);
    printf("bmp ac5 %u \n",bmpAC5);
    printf("bmp ac6 %u \n",bmpAC6);
    printf("bmp MC %d \n",bmpMC);
    printf("bmp md %d \n",bmpMD);

    printf("start measuremnet \n");
    start_read_bmp();




}


void readBmp(void *args){
    uint16_t temp;
    while(1){

        bmp180_read16(BMP180_READ_ADC_MSB_REG, (uint8_t *) &temp, 1+ ((uint8_t *) &temp) );
        float tf = 0.1*(((computeB5(temp, bmpAC5, bmpAC6, bmpMC, bmpMD)+8) >> 4 ));
        printf("temp %f \n",tf);
        start_read_bmp();
        vTaskDelay(pdMS_TO_TICKS(2000));


    }
}
void setup_temp(){
    temp_handle = NULL;
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));
}
void setup_timer(){
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = {
    .reload_count = 0, // counter will reload with 0 on alarm event
    .alarm_count = 1000000, // period = 1s @resolution 1MHz
    .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    gptimer_event_callbacks_t cbs = {
    .on_alarm = &button_isr, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}


void app_main(void) {
    setup_temp();

    if(mode_radar==1){
        setup_radar();
        setup_timer();


        vTaskDelay(pdMS_TO_TICKS(1000));

        xTaskCreate(radar_sensor,  "Radar sensor",  2048,  NULL, 2, NULL);
        xTaskCreate(convert_radar_sensor,  "Radar sensor reader",  2048,  NULL, 1, NULL);
        xTaskCreate(toggleLed,  "be a led",  2048,  NULL, 5, NULL);

    }
    if(mode_bmp == 1){
        setup_bmp();
        vTaskDelay(pdMS_TO_TICKS(1000));
        xTaskCreate(readBmp,  "BMP forever",  2048,  NULL, 5, NULL);

    }

}
