#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
// include esp_printf
#include "esp_log.h" // seems nice library, do not work

//QueueHandle_t queue; // q1
#define BUTTON_PIN 4 // q4,q5
typedef struct {
    QueueHandle_t in;
    QueueHandle_t out;
    SemaphoreHandle_t sem_in;
    SemaphoreHandle_t sem_out;
} mult_args_t;

uint32_t current_generator_id = 0; // for debugging purposes
mult_args_t mult_args; // shared arguments
SemaphoreHandle_t button_sem; // semapohre to avoid multiple press at the same time
SemaphoreHandle_t count_sem; // q5


/**
 * This task generates a sequence of integer, starting from one, one each second.
 * At startup, the task must print its name and CPU core id.
 * This task is a producer-only.
 *
 * @param[in]	args: The queue containing the integer stream
 *
 */

void generator(void *args) {
    mult_args_t * mult_gs = (mult_args_t*) args;

    xSemaphoreTake(mult_gs->sem_in,portMAX_DELAY); // use this semaphore to attribute a gererator id
    current_generator_id ++;
    uint32_t id = current_generator_id;
    char *name = pcTaskGetName(NULL);
    QueueHandle_t *queue = &mult_gs->in;
    int core = xPortGetCoreID();
    printf("%lu ) %s <%u> - Task start.\n",current_generator_id ,name, core);
    uint32_t i=0;
    xSemaphoreGive(mult_gs->sem_in);
    while (i<20) {
        i++;
        xQueueSend(*queue, &i, 1000);
        printf("%lu send i %lu \n", id,i);
        //wait 1 second
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("%lu ) generator end release semaphore \n",id);
    xSemaphoreGive(count_sem);
    vTaskDelete(NULL);
}

/**
 * This task consumes a sequence of integer.
 * At startup, the task must print its name and CPU core id.
 * This task is a consumer-only.
 *
 * @param[in]	args: The queue containing the integer stream
 *
 */
void sink(void *args) {
    mult_args_t * mult_gs = (mult_args_t*) args;
    char *name = pcTaskGetName(NULL);
    QueueHandle_t *queue = &mult_gs->out;
    int core = xPortGetCoreID();
    printf("%s <%u> - Task start.\n", name, core);
    xSemaphoreGive(mult_gs->sem_out);

    while (1) {
        uint32_t i;
        xQueueReceive(*queue, &i, portMAX_DELAY);
        printf("%s <%u> - Received %lu \n", name, core, i);
    }
}

/**
 * This task consumes a sequence of integer.
 * It then outputs the double of each integer.
 * At startup, the task must print its name and CPU core id.
 *
 * @param[in]	args: A structure containing the input queue and output queue.
 *
 */
void mult(void *args) {
    mult_args_t *mult_gs = (mult_args_t*) args;
    xSemaphoreTake(mult_gs->sem_out,portMAX_DELAY);

    char *name = pcTaskGetName(NULL);
    int core = xPortGetCoreID();
    printf("%s <%u> - Task start.\n", name, core);
    xSemaphoreGive(mult_gs->sem_in);

    while (1) {
        uint32_t i;
        xQueueReceive(mult_gs->in, &i, portMAX_DELAY);
        i = i*2;
        xQueueSend(mult_gs->out, &i, 1000);
    }

}

/**
 * Task controlled by the User button.
 *
 * In Part 3, it just print the message "Button pressed."
 *
 * In Part 8, it must also scheduler new `generator` tasks, up to 5.
 *
 * @param[in]	args: Unused
 */
void button_task(void *args) {
    if(xSemaphoreTake(button_sem, 0) == pdTRUE){
        printf("Button pressed.\n");
        // take count semaphore and create new task
        if(xSemaphoreTake(count_sem, ( TickType_t ) 10 ) == pdTRUE){
            printf("Add new generator \n");
            xTaskCreatePinnedToCore(generator, "generator", 2048, &mult_args, 1, NULL,0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // max 1 press per second
        xSemaphoreGive(button_sem);

    }
    vTaskDelete(NULL);

}

/**
 * ISR to be attached to the button GPIO IRQ.
 *
 * This ISR must activate the `button_task` task.
 */
static void IRAM_ATTR button_isr(void *args) {
    xTaskCreatePinnedToCore(button_task, "button_task", 2048, NULL, 2, NULL,1);
}

/**
 * This function set-up all features used for this exercise.
 * @return	0 if no error, else 1
 */
static inline int setup(void) {
    //question 1-3
    mult_args.in = xQueueCreate(40, sizeof(uint32_t));
    mult_args.out = xQueueCreate(40, sizeof(uint32_t));
    mult_args.sem_in = xSemaphoreCreateBinary();
    mult_args.sem_out= xSemaphoreCreateBinary();

    // question 4.5 - 5
    button_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(button_sem); // at the start , the button should be able to be pressed
    count_sem = xSemaphoreCreateCounting(5,5); // make 5 free spaces out of 5 for the generators

    // gpio config setup for button
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Attach the button ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL);



    if (mult_args.in == NULL || mult_args.out == NULL) {
        printf("Queue creation failed.\n");
        return 1;
    }


    return 0;
}

/**
 * Main function called by the ESP-IDF SDK.
 */
void app_main(void) {

    printf("Hello from ESP-IDF.\n");

    if (setup()) {
        printf("Exit.\n");
        return;
    }

    printf("Initial setup finished. Starting tasks.\n");
    xTaskCreatePinnedToCore(sink, "sink", 2048, &mult_args, 1, NULL,0);
    xTaskCreatePinnedToCore(mult, "mult", 2048, &mult_args, 1, NULL, 1);

}
