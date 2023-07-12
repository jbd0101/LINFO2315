#include "mdk.h"
#include <setjmp.h>
#include "include/sched.h"
#define LED 35
#define TRIG_PIN 47
#define ECHO_PIN 48
#define SIGNAL 19
extern void _xtos_set_interrupt_handler(int i, void* func, void *arg);
//PROVIDE(_xtos_set_interrupt_handler = 0x40001c20);


void setup_echo(void) {
    gpio_output(TRIG_PIN);
    gpio_input(ECHO_PIN);
    // TODO
}

void setup_led(void) {
    // TODO
    gpio_output(LED);

}

/** #################### Part 1 ###################### */

void task1_callback(void) {
    while(1){
        printf("task 1 \n");
    }
    // TODO
}

void task2_callback(void) {
    while(1){

        gpio_write(LED,1);
        delay_ms(1000);
        gpio_write(LED,0);
        delay_ms(1000);
    }

    // TODO
}

void task3_callback(void) {
    while(1){
        gpio_write(TRIG_PIN, 0);
        delay_us(2);
        gpio_write(TRIG_PIN, 1);
        delay_us(10);
        gpio_write(TRIG_PIN, 0);

        uint64_t start=0;
        if(busy_wait(ECHO_PIN,1,100000)==1){
            printf("impossible de detecteur le depart \n");
        }
        start = uptime_us();
        if(busy_wait(ECHO_PIN,0,100000)==1){
            printf("impossible de detecteur une fin \n");

        }
        int range = (int)(uptime_us()-start);
        range = (range) *340/20000;
        printf("distance: %d \n",range);
    }

    // TODO
}

/** #################### Part 2 ###################### */

// TODO: Define the Timer Group registers

// TODO: Define the Interrupt registers

// TODO: Define the interrupt number that you will use
#define SIGNAL 19

/**
 * @brief Simple panic ISR. Should print "Panic!" then do nothing, i.e., never return.
 */
void my_isr(void) {
    printf("panic! \n");


    TIMG_INT_CLR_TIMERS_REG |= BIT(0) | BIT(1);
    TIMG_T0CONFIG_REG |= BIT(9) | BIT(30);
    TIMG_T0LOADLO_REG = 0;
    TIMG_T0LOADHI_REG = 0;
    TIMG_T0LOAD_REG   = 168;
    TIMG_T0ALARMLO_REG = 50000000;
    TIMG_T0ALARMHI_REG = 0;
    TIMG_INT_ENA_TIMERS_REG |= BIT(0);
    TIMG_T0CONFIG_REG |= BIT(10);
    TIMG_T0CONFIG_REG |= BIT(31);

}

/**
 * @brief Setup a one-shot Timer Interrupt
 */
//static void setup(void) {
    // TODO
//}

static void setup(void) {
    // 1. use XTAL_CLK and increase the timer counter
    TIMG_T0CONFIG_REG |= BIT(9) | BIT(30);

    // Set the timer’s starting value by writing the starting value to TIMG_Tx_LOAD_LO and
    // TIMG_Tx_LOAD_HI, then reloading it into the timer by writing any value to TIMG_TxLOAD_REG.
    TIMG_T0LOADLO_REG = 0;
    TIMG_T0LOADHI_REG = 0;
    TIMG_T0LOAD_REG   = 168;

    // 2. Configure the alarm value by setting TIMG_TxALARMLO_REG and TIMG_TxALARMHI_REG.
    TIMG_T0ALARMLO_REG = 100000000;
    TIMG_T0ALARMHI_REG = 0;
    // Enable interrupt by setting TIMG_Tx_INT_ENA
    TIMG_INT_ENA_TIMERS_REG |= BIT(0);

    // 3. Disable auto reload by clearing TIMG_Tx_AUTORELOAD
    TIMG_T0CONFIG_REG &= ~BIT(29);

    // allocate Timer0 of TG0 to the interrupt signal SIGNAL on Core 0.
    INTERRUPT_CORE0_TG_T0_INT_MAP_REG = SIGNAL;

    _xtos_set_interrupt_handler(SIGNAL, my_isr, NULL);
    asm volatile ("wsr %0, intenable; rsync" : : "r"(BIT(SIGNAL)) : );
}

/** #################### Main ###################### */

int main(void) {
    // Disable WatchDog Timers
    wdt_disable();
    setup_led();        // Enable the LED GPIO

    printf("Hello from ESP32s3\n");
    setup_echo();       // Setup GPIOs to interact with the HC-SR04 sensor

    setup();

    // 4. Start the alarm by setting TIMG_Tx_ALARM_EN.
    TIMG_T0CONFIG_REG |= BIT(10);

    // Start the timer by setting TIMG_Tx_EN.
    TIMG_T0CONFIG_REG |= BIT(31);

   /* 
    sched_t mySched = schedInit();
    struct task t1 = {&task1_callback};
    taskAdd(mySched,&t1);
    struct task t2 = {&task2_callback};
    taskAdd(mySched,&t2);
    */


    /*struct task t3 = {&task3_callback};
    taskAdd(mySched,&t3);
    schedStart(mySched);
    */

    /* TODO: Part 1
     * 1. Initialize your scheduler
     * 2. Register the tasks
     * 3. Launch the scheduler
     */

    /* TODO: Part 2
     * - Setup your one-shot timer-based interrupt.
     */

    return 0;
}
