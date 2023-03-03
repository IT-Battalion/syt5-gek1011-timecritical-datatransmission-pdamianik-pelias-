#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"

#define BUFFER_SIZE 0x100
#define ERROR_CODE 0xFF

#define PICO_DEFAULT_LED_PIN 25

#define SCK 2 //sck
#define CSn 5 //cs
#define Rx 4 //miso
#define Tx 3 //mosi

// Initialize the traffic light pins and direction
static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}

static void init_spi(void) {
    // Initialize SPI interface
    spi_init(spi0, 1000 * 1000);  // Set clock frequency to 100 MHz
    spi_set_slave(spi0, true);
    gpio_set_function(Rx, GPIO_FUNC_SPI);
    gpio_set_function(CSn, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(Tx, GPIO_FUNC_SPI);
} 


int main() {
    init_pins();
    init_spi();

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }
}