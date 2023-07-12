# Embedded Rust

**PREREQUISTES**

You should read the [documentation introducing the usage of Rust on ESP platforms](https://esp-rs.github.io/book/).

## Part 1: Bare-metal Rust on ESP32

### Objective:
In this exercise, you will create a Rust project that will run on a Heltec ESP32 LoRa board. The goal is to control an LED connected to GPIO25 of the board, making it blink with a 1-second interval. You will use the esp32-hal library to interact with the board's hardware, as well as the Rust `no_std` and `no_main` attributes.

### Requirements:

**Read Section 5.2 of the [documentation](https://esp-rs.github.io/book/writing-your-own-application/no-std-applications/index.html)**

1. Create a new Rust project with the `no_std` and `no_main` attributes. This will disable the Rust standard library and the default entry point for the program.
2. Use the esp32_hal library to initialize and manage the ESP32 peripherals. You will need to interact with the following peripherals:
	- Clock Control
	- Real-Time Clock (RTC)
	- Timer Groups (TIMG0 and TIMG1)
	- GPIO25 as output
3. Disable the RTC and TIMG watchdog timers.
4. Configure GPIO25 as an output pin, and set its initial state to high.
5. Implement a loop that toggles the state of the LED connected to GPIO25 every 500 milliseconds, resulting in a 1-second blinking interval.

### Hints:

- To set the LED state to high, use the `set_high()` method on the LED object.
- To toggle the LED state, use the `toggle()` method on the LED object.
- Use the `delay_ms()` method from the Delay peripheral to implement the delay between LED state changes.

## Part 2: ESP-IDF with Rust on ESP32

**Read Section 5.3 of the [documentation](https://esp-rs.github.io/book/writing-your-own-application/std-applications/index.html)**

### Objective:
In this exercise, you will create a Rust project that will run on a Heltec ESP32 LoRa board. The goal is to control an LED connected to GPIO25 of the board, making it blink the SOS Morse code pattern (three short blinks, three long blinks, three short blinks) with appropriate intervals between blinks and repeats. You will use the esp_idf_hal library to interact with the board's hardware.

### Requirements:

1. Create a new Rust project and add the esp_idf_hal and esp_idf_sys libraries to the dependencies.
2. Initialize the ESP32 peripherals using the Peripherals struct from the esp_idf_hal library.
3. Configure GPIO25 as an output pin using the PinDriver struct and the output() method.
4. Implement an infinite loop that continuously blinks the SOS Morse code pattern:Blink the LED three times with short intervals (300 ms) to represent the letter 'S' in Morse code.Blink the LED three times with long intervals (1000 ms) to represent the letter 'O' in Morse code.Blink the LED three times with short intervals (300 ms) to represent the letter 'S' in Morse code.Wait for a pause (3000 ms) before repeating the pattern.
5. Create separate functions to handle short blinks, long blinks, and the complete SOS Morse code pattern.

### Hints:

- To set the LED state to high, use the `set_high()` method on the LED object.
- To set the LED state to low, use the `set_low()` method on the LED object.
- Use the `thread::sleep()` function from the `std::thread` module to implement the delays between LED state changes and between SOS Morse code pattern repetitions.

