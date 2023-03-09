#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"

#define ERROR_CODE 0xF
#define PICO_DEFAULT_LED_PIN 25

#define PIN_SCK 2 //sck
#define PIN_CS 5 //cs
#define PIN_RX 4 //miso
#define PIN_TX 3 //mosi

TaskHandle_t tsDataHandler = NULL;
TaskHandle_t tsBlinker = NULL;
TickType_t timestamp = 0;

static void init_pins(void);
static void init_spi(void);
static void tBlinker(void*);
static void tDataHandler(void*);

int main() {
    init_pins();
    init_spi();

    xTaskCreate(tDataHandler, "dataHandler", 1024, NULL, 1, &tsDataHandler);
    xTaskCreate(tBlinker, "blinkHandler", 1024, NULL, 1, &tsBlinker);
    vTaskStartScheduler();

    while (true)
    {
        //never reached
    }
}

static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static void init_spi(void) {
    spi_init(spi0, 1000 * 1000);
    spi_set_slave(spi0, false);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
} 

void tBlinker(void* p) {
    while (true) {
        vTaskDelay(30);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(30);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}

void tDataHandler(void* p) {
    uint8_t buffer[1];
    while (true) {
        gpio_put(PIN_CS, 0);
        if (spi_is_readable(spi0))
        {
            timestamp = xTaskGetTickCount();
            spi_read_blocking(spi0, 0, buffer, sizeof(buffer));
        }
        
        if ((xTaskGetTickCount() - timestamp) > pdMS_TO_TICKS(60))
        {
            if (spi_is_writable(spi0))
            {
                uint8_t send[1] = {ERROR_CODE};
                spi_write_blocking(spi0, send, sizeof(send));
            }
        }
        gpio_put(PIN_CS, 1);
        vTaskDelay(30);
    }
}