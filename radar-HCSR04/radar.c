#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#define TRIG 46
#define ECHO 45
#define LED 35
#define BTN 0
gptimer_handle_t gptimer = NULL;
QueueHandle_t xQueue;
SemaphoreHandle_t sem_led;

void button_isr(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sem_led,&xHigherPriorityTaskWoken);
}

void setup_radar(){
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << TRIG) | (1ULL << LED);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << ECHO);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);


	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << BTN);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	
	// Attach the button ISR handler
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BTN, button_isr, NULL);


	gptimer_config_t timer_config = {
	.clk_src = GPTIMER_CLK_SRC_DEFAULT,
	.direction = GPTIMER_COUNT_UP,
	.resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
	ESP_ERROR_CHECK(gptimer_enable(gptimer));
	ESP_ERROR_CHECK(gptimer_start(gptimer));
	ESP_ERROR_CHECK(gptimer_stop(gptimer));
	ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer,0));
	sem_led = xSemaphoreCreateBinary();
	
}

uint8_t busy_wait(uint8_t pin, uint8_t level,uint32_t timeout){
	while(gpio_get_level(pin) != level){
		timeout --;
		if(timeout==0){
			return 0;
		}

	}
	return 1;
}

void led_task(void *args){
	uint8_t status = 1;
	while(1){
		xSemaphoreTake(sem_led, portMAX_DELAY);
		status = !status;
		gpio_set_level(LED,status);

	}
}


void read_bmp(void *args){
	uint8_t error = 0;
	while(1){
		error = 0;
		gpio_set_level(TRIG, 0);
		vTaskDelay(pdMS_TO_TICKS(1));
		gpio_set_level(TRIG, 1);
		vTaskDelay(pdMS_TO_TICKS(1));
		gpio_set_level(TRIG, 0);
		if(busy_wait(ECHO,1,100000)==0){
			printf("une erreur est survenue \n");
			error = 1;
		}
		ESP_ERROR_CHECK(gptimer_start(gptimer));
		// Retrieve the timestamp at any time
		if(busy_wait(ECHO,0,100000)==0){
			printf("une erreur est survenue \n");
			error =1;
		}

		gptimer_stop(gptimer);
		uint64_t time;
		ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &time));
		ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer,0));

		if(error==0){
			xQueueSend(xQueue,&time,portMAX_DELAY);
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
void convert_and_print(void *args){
	uint64_t data=0;
	while(1){
		xQueueReceive(xQueue,&data,portMAX_DELAY);
		printf("centimer: %llu \n",data/58);


	}
}

void app_main(){
	setup_radar();
	xQueue = xQueueCreate( 20, sizeof(uint64_t));

	vTaskDelay(pdMS_TO_TICKS(500));
	xTaskCreate(read_bmp,"read bmp", 2048, NULL, 5, NULL);
	xTaskCreate(convert_and_print,"convert_and_print", 2048, NULL, 5, NULL);
	xTaskCreate(led_task,"LED", 2048, NULL, 6, NULL);
	
}