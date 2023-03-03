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

TaskHandle_t tsDataHandler = NULL;
TaskHandle_t tsBlinker = NULL;

// Initialize the traffic light pins and direction
static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static void init_spi(void) {
    // Initialize SPI interface
    spi_init(spi0, 1000 * 1000);  // Set clock frequency to 100 MHz
    spi_set_slave(spi0, false);
    gpio_set_function(Rx, GPIO_FUNC_SPI);
    gpio_set_function(CSn, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(Tx, GPIO_FUNC_SPI);
} 

void blink(void* p) {
    for ( ;; ) {
        vTaskDelay(1500);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(1500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}
TickType_t timestamp = 0;
void dataHandler(void* p) {
    uint8_t buffer[10];
    for ( ;; ) {
        // Pull CS low to select the device
        gpio_put(CSn, 0);
        
        // Read data from the device
        if (spi_is_readable(spi0))
        {
            timestamp = xTaskGetTickCount();
            spi_read_blocking(spi0, 0, buffer, sizeof(buffer));
        }
        
        // Process the received data here
        if ((xTaskGetTickCount() - timestamp) > 60)
        {
            if (spi_is_writable(spi0))
            {
                uint8_t send[] = {ERROR_CODE};
                spi_write_blocking(spi0, send, sizeof(send));
            }
        }

        // Pull CS high to deselect the device
        gpio_put(CSn, 1);
        vTaskDelay(30);
    }
}

int main() {
    init_pins();
    init_spi();

    xTaskCreate(dataHandler, "dataHandler", 1024, NULL, 1, &tsDataHandler);
    xTaskCreate(blink, "blinkHandler", 1024, NULL, 1, &tsBlinker);
    vTaskStartScheduler();

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }
}