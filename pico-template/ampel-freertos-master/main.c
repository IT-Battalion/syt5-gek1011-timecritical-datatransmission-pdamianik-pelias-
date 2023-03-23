#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"

#define ERROR_CODE 0xF
#define PICO_DEFAULT_LED_PIN 25

#define PIN_SCK 2 //sck
#define PIN_CS 1 //cs
#define PIN_RX 0 //miso
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

    xTaskCreate(tDataHandler, "dataHandler", 1024, NULL, 2, &tsDataHandler);
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

dma_channel_config dma_cfg;

static void init_spi(void) {
    spi_init(spi0, 1000*1000);
    spi_set_slave(spi0, false);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    dma_cfg = dma_channel_get_default_config(spi_get_index(spi0));
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
    channel_config_set_dreq(&dma_cfg, spi_get_index(spi0) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, true);
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
        const uint dma_channel = dma_claim_unused_channel(true);
        dma_channel_start(dma_channel);
        dma_channel_configure(
            dma_channel, 
            &dma_cfg,
            buffer, 
            &spi_get_hw(spi0)->dr,  
            sizeof(buffer), 
            true  
        );

        dma_channel_wait_for_finish_blocking(dma_channel);
        
        if ((xTaskGetTickCount() - timestamp) > pdMS_TO_TICKS(60))
        {
            uint8_t send[1] = {ERROR_CODE};
            const uint dma_write = dma_claim_unused_channel(true);
            dma_channel_start(dma_write);
            dma_channel_configure(
                dma_write, 
                &dma_cfg,
                &spi_get_hw(spi0)->dr,
                send,
                sizeof(send), 
                true 
            );
            dma_channel_wait_for_finish_blocking(dma_write);
            dma_channel_unclaim(dma_write);
        }
        dma_channel_unclaim(dma_channel);
        vTaskDelay(30);
    }
}