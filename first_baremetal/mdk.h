#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BIT(x) ((uint32_t) 1U << (x))
#define REG(x) ((volatile uint32_t *) (x))

// See Table 4-3 Section 4.3.5.1 of ESP32s3 reference manual

#define S3_UART 0x60000000
#define S3_SPI1 0x60002000
#define S3_SPI0 0x60003000
#define S3_GPIO 0x60004000
#define S3_RTC_CNTL 0x60008000
#define S3_SENS	0x60008800
#define S3_IO_MUX 0x60009000
#define S3_I2S0 0x6000f000

#define S3_UART1 0x60010000
#define S3_I2C0 0x60013000
#define S3_UHCI0 0x60014000
#define S3_RMT 0x60016000
#define S3_PULSE 0x60017000
#define S3_LEDC 0x60019000
#define S3_MOTOR_PWM0 0x6001e000
#define S3_TIMERGROUP0 0x6001f000

#define S3_TIMERGROUP1 0x60020000
#define S3_RTC_SLOW 0x60021000
#define S3_SYSTIMER 0x60023000
#define S3_SPI2 0x60024000
#define S3_SPI3 0x60025000
#define S3_APB_CTRL 0x60026000
#define S3_I2C1 0x60027000
#define S3_SD_MMC 0x60028000
#define S3_TWAI 0x6002b000
#define S3_MOTOR_PWM1 0x6002c000
#define S3_I2S1 0x6002d000
#define S3_UART2 0x6002e000

#define S3_AES 0x6003a000
#define S3_SHA 0x6003b000
#define S3_RSA 0x6003c000
#define S3_DIGITAL_SIGNATURE 0x6003d000
#define S3_HMAC 0x6003e000
#define S3_GDMA 0x6003f000

#define S3_APB_SARADC 0x60040000

#define S3_SYSTEM 0x600c0000
#define S3_SENSITIVE 0x600c1000
#define S3_INTERRUPT 0x600c2000
//#define C3_EXTMEM 0x600c4000
//#define C3_MMU_TABLE 0x600c5000
#define S3_ASSIST_DEBUG 0x600ce000

#define S3_WORLD_CNTL 0x600d0000

//#define C3_DEDICATED_GPIO 0x600cf000
//#define C3_DPORT_END 0x600d3FFC
//#define C3_FE2 0x60005000
//#define C3_FE 0x60006000

//#define C3_RTCCNTL 0x60008000
//#define C3_EFUSE 0x60008800
//#define C3_NRX 0x6001CC00
//#define C3_BB 0x6001D000

//#define C3_SYSCON 0x60026000
//#define C3_AES_XTS 0x600CC000

enum { GPIO_OUT_EN = 8, GPIO_OUT_FUNC = 341, GPIO_IN_FUNC = 85 };

// Perform `count` "NOP" operations
static inline void spin(volatile unsigned long count) {
  while (count--) asm volatile("nop");
}

static inline uint64_t systick(void) {
  REG(S3_SYSTIMER)[1] = BIT(30);  // TRM 10.5
  spin(1);
  return ((uint64_t) REG(S3_SYSTIMER)[16] << 32) | REG(S3_SYSTIMER)[17];
}

static inline uint64_t uptime_us(void) {
  return systick() >> 4;
}

static inline void delay_us(unsigned long us) {
  uint64_t until = uptime_us() + us;
  while (uptime_us() < until) spin(1);
}

static inline void delay_ms(unsigned long ms) {
  delay_us(ms * 1000);
}

#define RTC_CNTL_RTC_WDTWPROTECT_REG 	REG(S3_RTC_CNTL)[44]
#define RTC_CNTL_RTC_WDTCONFIG0_REG 	REG(S3_RTC_CNTL)[38]

#define RTC_CNTL_RTC_SWD_CONF_REG	REG(S3_RTC_CNTL)[45]
#define RTC_CNTL_RTC_SWD_WPROTECT_REG	REG(S3_RTC_CNTL)[46]
#define RTC_CNTL_RTC_SW_CPU_STALL_REG	REG(S3_RTC_CNTL)[47]

#define TIMG0_WDTCONFIG0_REG	REG(S3_TIMERGROUP0)[18]
#define TIMG1_WDTCONFIG0_REG	REG(S3_TIMERGROUP1)[18]

static inline void wdt_disable(void) {

  // Disable WDT. TRM 13.2.2.3
  RTC_CNTL_RTC_WDTWPROTECT_REG = 0x50d83aa1;  // Disable write protection
  RTC_CNTL_RTC_WDTCONFIG0_REG = 0;  // Disable RTC WDT
  //REG(S3_RTC_CNTL)[35] = 0;  // Disable
  RTC_CNTL_RTC_WDTWPROTECT_REG = 0x0;  // Enable write protection

  // Enable Super WDT auto-feed
  RTC_CNTL_RTC_SWD_WPROTECT_REG = 0x8F1D312A;	// Disable write protection
  RTC_CNTL_RTC_SWD_CONF_REG |= BIT(31);	// Enable auto-feed
  RTC_CNTL_RTC_SW_CPU_STALL_REG = 0;	// Disable Super WDT ability to stall CPUs

  //REG(S3_TIMERGROUP0)[63] &= ~BIT(9);  // TIMG_REGCLK -> disable TIMG_WDT_CLK
  TIMG0_WDTCONFIG0_REG = 0;         // Disable TG0 WDT
  TIMG1_WDTCONFIG0_REG = 0;         // Disable TG1 WDT
}

/*static inline void wifi_get_mac_addr(uint8_t mac[6]) {
  uint32_t a = REG(C3_EFUSE)[17], b = REG(C3_EFUSE)[18];
  mac[0] = (b >> 8) & 255, mac[1] = b & 255, mac[2] = (uint8_t) (a >> 24) & 255;
  mac[3] = (a >> 16) & 255, mac[4] = (a >> 8) & 255, mac[5] = a & 255;
}*/

#define SYSTEM_CPU_PER_CONF_REG	REG(S3_SYSTEM)[4]
#define SYSTEM_SYSCLK_CONF_REG	REG(S3_SYSTEM)[24]
#define ROM_ETS_UPDATE_CPU_FREQUENCY 0x40043164

static inline void soc_init(void) {
  // Init clock. TRM 6.2.4.1
  SYSTEM_CPU_PER_CONF_REG &= ~3U;	// Reset SYSTEM_CPU_PER_CONF_REG
  SYSTEM_CPU_PER_CONF_REG |= BIT(0) | BIT(2);	// Enable SYSTEM_CPUPERIOD_SEL and SYSTEM_PLL_FREQ_SEL
  SYSTEM_SYSCLK_CONF_REG = (40U << 12) | BIT(10);	// Select PLL_CLK in SYSTEM_SOC_CLK_SEL. Table 6-2 TRM.
  //((void (*)(int)) ROM_ETS_UPDATE_CPU_FREQUENCY)(160);  // ets_update_cpu_frequency(160)
}

// API GPIO
#define OFFSET(offset)	offset / 4

#define GPIO_PIN(pin)	pin < 32 ? pin : pin - 32

#define GPIO_OUT_REG	REG(S3_GPIO)[OFFSET(0x4)]
#define GPIO_OUT_W1TS_REG	REG(S3_GPIO)[OFFSET(0x8)]
#define GPIO_OUT_W1TC_REG	REG(S3_GPIO)[OFFSET(0xc)]

#define GPIO_OUT1_REG	REG(S3_GPIO)[OFFSET(0x10)]
#define GPIO_OUT1_W1TS_REG	REG(S3_GPIO)[OFFSET(0x14)]
#define GPIO_OUT1_W1TC_REG	REG(S3_GPIO)[OFFSET(0x18)]

#define GPIO_ENABLE_REG	REG(S3_GPIO)[OFFSET(0x20)]
#define GPIO_ENABLE_W1TS_REG	REG(S3_GPIO)[OFFSET(0x24)]
#define GPIO_ENABLE_W1TC_REG	REG(S3_GPIO)[OFFSET(0x28)]

#define GPIO_ENABLE1_REG	REG(S3_GPIO)[OFFSET(0x2c)]
#define GPIO_ENABLE1_W1TS_REG	REG(S3_GPIO)[OFFSET(0x30)]
#define GPIO_ENABLE1_W1TC_REG	REG(S3_GPIO)[OFFSET(0x34)]

#define GPIO_FUNCx_OUT_SEL_CFG_REG(x) REG(S3_GPIO)[OFFSET(0x554 + 0x4*x)]

#define GPIO_IN_REG	REG(S3_GPIO)[15]
#define GPIO_IN1_REG	REG(S3_GPIO)[16]

static inline void gpio_output_enable(int pin, bool enable) {
    int final_pin = GPIO_PIN(pin);
    if (pin < 32) {
	GPIO_ENABLE_REG &= ~BIT(final_pin);
	GPIO_ENABLE_REG |= (enable ? 1U : 0U) << final_pin;
    } else {
	GPIO_ENABLE1_REG &= ~BIT(final_pin);
	GPIO_ENABLE1_REG |= (enable ? 1U : 0U) << final_pin;
    }
}

static inline void gpio_output(int pin) {
  GPIO_FUNCx_OUT_SEL_CFG_REG(pin) = 0x100;  // Simple out, TRM 6.5.3
  gpio_output_enable(pin, 1);
}

static inline void gpio_write(int pin, bool value) {
/*    if(pin>45){
      printf("invalid pin >45 \n");
      return;
    }*/
    if(pin<32){

      if(pin>=22 && pin <= 25){
        printf("unauthorized gpio\n");
        return;
      }
      if(value){
        GPIO_OUT_W1TS_REG |= BIT(pin);

      }else{
        GPIO_OUT_W1TC_REG |= BIT(pin);
      }
     // GPIO_OUT1_REG=0;
    }else{
      if(value){
        GPIO_OUT1_W1TS_REG |= BIT(pin-32);

      }else{
        GPIO_OUT1_W1TC_REG |= BIT(pin-32);
      }
    }
    // TODO. See https://inginious.info.ucl.ac.be/course/LINFO2315/gpio_write-implem
}

#define IO_MUX_n_REG(n)	REG(S3_IO_MUX)[OFFSET(0x10 + 0x4 * n)]

static inline void gpio_input(int pin) {
  gpio_output_enable(pin, 0);                 // Disable output
  IO_MUX_n_REG(pin) = BIT(9) | BIT(7);  // Enable pull-up in IO_MUX_<pin>_REG
}

static inline bool gpio_read(int pin) {
  return ((pin < 32) ? GPIO_IN_REG : GPIO_IN1_REG ) & BIT(GPIO_PIN(pin)) ? 1 : 0;
}

static inline int busy_wait(int gpio, bool level, int timeout) {
    // TODO. See https://inginious.info.ucl.ac.be/course/LINFO2315/gpio-control-4
  gpio_input(gpio);
  while(gpio_read(gpio) != level) {
    if (timeout == 0) return 1;
    spin(1);
    timeout--;
  }
  return 0;
}
