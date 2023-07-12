#include "mdk.h"

/** ############################ PART 1 ############################################# */

#define SENS_SAR_PERI_CLK_GATE_CONF_REG	REG(S3_SENS)[65]
#define SENS_SAR_TSENS_CTRL_REG REG(S3_SENS)[20]
// TODO Define the registers required to handle the embedded temperature sensor

/**
 * @brief Enable the embedded temperature sensor.
 */
void setup_temp_sensor(void) {
    SENS_SAR_PERI_CLK_GATE_CONF_REG |= BIT(29);

    SENS_SAR_TSENS_CTRL_REG |= BIT(22) | BIT(23);
    delay_ms(500);
    SENS_SAR_TSENS_CTRL_REG |= BIT(24);
}

/**
 * @brief Transform a raw value from the temperature sensor in a calibrated value
 *
 * @param[in]	raw_value	The raw value from the sensor
 * @return			The calibrated value
 */
static inline double calibrate(int raw_value) {
    // TODO
    return 0.4386 * raw_value - 27.88 * 0 - 20.52;
}

/**
 * @brief Poll once the temperature sensor and show the calibrated result on STDOUT.
 */
void poll_temp_sensor(void) {
    // TODO: get the raw value from the sensor register

    // TODO: show the calibrated value in hexadecimal on STDOUT
    while((SENS_SAR_TSENS_CTRL_REG>> 8 )& 0x1 ==0){
    }
    printf("boulp %x \n",SENS_SAR_TSENS_CTRL_REG);
    int temp = SENS_SAR_TSENS_CTRL_REG & 0xff;
    printf("temperature : %d \n", (int) calibrate(temp));

}


/** ############################ PART 2 ############################################# */

// TODO Define LED GPIO
#define LED 35
#define TRIG_PIN 47
#define ECHO_PIN 48
/**
 * @brief Enable the LED GPIO as an output
 */
void setup_led(void) {
    // TODO
    gpio_output(LED);

}
    
/**
 * @brief Lights the LED up for 125[ms]
 */
void toggle_led(void) {
    // TODO
    gpio_write(LED,1);
    delay_ms(1000);
    gpio_write(LED,0);
    delay_ms(1000);

}

/** ############################ PART 3 ############################################# */

// TODO Define the TRIG and ECHO GPIO pins


/**
 * @brief Set-up the GPIO pins for the HC-SR04 sensor
 */
void setup_echo(void) {
    gpio_output(TRIG_PIN);
    gpio_input(ECHO_PIN);
/*    while(1){
        printf("data: %d \n",gpio_read(ECHO_PIN));
        delay_ms(100);
    }*/
    // TODO
}

/**
 * @brief Poll the HC-SR04 sensor
 *
 * @param[out]	range	The raw range value computed by the sensor
 * @return		Status value, 0 if no error else 1
 */
int poll_echo(int *range) {

    // TODO Send trigger signal to HC-SR04

    // TODO Compute the range value
    gpio_write(TRIG_PIN, 0);
    delay_us(2);
    gpio_write(TRIG_PIN, 1);
    delay_us(10);
    gpio_write(TRIG_PIN, 0);

    uint64_t start=0;
    if(busy_wait(ECHO_PIN,1,100000)==1){
        printf("impossible de detecteur le depart \n",start);
        return -1;
    }
    start = uptime_us();
    if(busy_wait(ECHO_PIN,0,100000)==1){
        printf("impossible de detecteur une fin \n",start);

        return -1;
    }
    *range = (int)(uptime_us()-start);
    *range = (*range) *340/20000;
    //////printf("distance: %d \n",*range);
    return 0;
}

/** ############################ Board setup and I/O loop ########################### */

int main(void) {
    wdt_disable();		// Disable the hardware WatchDog timers
    printf("Hello from ESP32s3\n");

    setup_temp_sensor();	// Enable the temperature sensor
    setup_led();		// Enable the LED GPIO
    setup_echo();		// Setup GPIOs to interact with the HC-SR04 sensor
    
    int range = 0;
    int status = 0;

    // A simple I/O loop.
    while (1) {
	poll_temp_sensor();
	toggle_led();
	
	range = 0;		// Reset range value
	status = poll_echo(&range);
	printf("range %x %x\n", status, range);

	delay_ms(100);
    }
}
