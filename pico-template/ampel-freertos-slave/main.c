#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"

#define PIN_SCK 18 //sck
#define PIN_CS 17 //cs
#define PIN_RX 21 //miso
#define PIN_TX 19 //mosi
#define ERROR_CODE 0xF
#define LIGHT_RED    0
#define LIGHT_YELLOW 1
#define LIGHT_GREEN  2
#define PICO_DEFAULT_LED_PIN 25
#define DURATION_YELLOW_BLINKING 500
#define DURATION_YELLOW          4000
#define DURATION_RED             5000
#define DURATION_RED_YELLOW      4000
#define DURATION_GREEN           5000
#define DURATION_GREEN_BLINKING  500
#define STATUS_RED            0xE
#define STATUS_RED_YELLOW     0xD
#define STATUS_GREEN          0x2
#define STATUS_GREEN_BLINKING 0x5
#define STATUS_YELLOW         0x8
#define STATUS_YELLOW_BLINKING 0x1
#define NUM_STATES 7

typedef enum {
    STATE_YELLOW_BLINKING,
    STATE_YELLOW,
    STATE_RED,
    STATE_RED_YELLOW,
    STATE_GREEN,
    STATE_GREEN_BLINKING,
    STATE_ERROR
} State;

typedef struct {
    State state;
    uint8_t status;
} StateStatusPair;

static StateStatusPair stateStatusMap[] = {
    { STATE_YELLOW_BLINKING, STATUS_YELLOW_BLINKING },
    { STATE_YELLOW, STATUS_YELLOW },
    { STATE_RED, STATUS_RED },
    { STATE_RED_YELLOW, STATUS_RED_YELLOW },
    { STATE_GREEN, STATUS_GREEN },
    { STATE_GREEN_BLINKING, STATUS_GREEN_BLINKING },
    { STATE_ERROR, ERROR_CODE},
};

TaskHandle_t tsDataTransmitter = NULL;
TaskHandle_t tsStateHandler = NULL;

typedef void (*State_Function)(void);

static void state_yellow_blinking(void);
static void state_yellow(void);
static void state_red(void);
static void state_red_yellow(void);
static void state_green(void);
static void state_green_blinking(void);
static void state_error(void);

static State_Function state_functions[NUM_STATES] = {
    state_yellow_blinking,
    state_yellow,
    state_red,
    state_red_yellow,
    state_green,
    state_green_blinking,
    state_error,
};

static const int pins[] = {
    LIGHT_RED,
    LIGHT_YELLOW,
    LIGHT_GREEN,
};

State state = STATE_ERROR;

static uint8_t getStatusForState(State state) {
    for (int i = 0; i < NUM_STATES; i++) {
        if (stateStatusMap[i].state == state) {
            return stateStatusMap[i].status;
        }
    }
    return -1;
}

static void init_pins(void);
static void init_spi(void);
static void tTransmitState(void*);
static void tHandleState(void*);

int main() {
    init_pins();
    init_spi();

    if (!watchdog_caused_reboot()) {
        state = STATE_YELLOW_BLINKING;
    } 

    watchdog_enable(600, false);

    xTaskCreate(tTransmitState, "dataTransmitter", 1024, NULL, 1, &tsDataTransmitter);
    xTaskCreate(tHandleState, "stateHandler", 1024, NULL, 1, &tsStateHandler);
    vTaskStartScheduler();

    while (true)
    {
        //never reached
    }
}

static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}

static void init_spi(void) {
    spi_init(spi0, 1000 * 1000); 
    spi_set_slave(spi0, true);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
} 

static void state_error(void) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    gpio_put(pins[LIGHT_YELLOW], 1);
    vTaskDelay(DURATION_YELLOW_BLINKING);
    gpio_put(pins[LIGHT_YELLOW], 0);
    vTaskDelay(DURATION_YELLOW_BLINKING);
}

void tTransmitState(void* p) {
    uint8_t rx_buffer[10];
    while (true) {
        if (state != STATE_ERROR)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);

            uint8_t send_buf[10] = {getStatusForState(state)};
            int len = 0;
            if (spi_is_writable(spi0))
            {
                len = spi_write_read_blocking(spi0, send_buf, rx_buffer, sizeof(send_buf));
            }

            if (rx_buffer[0] != ERROR_CODE && len > 0)
            {
                watchdog_update();
                vTaskDelay(10);
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
                vTaskDelay(10);
            }
        } else {
            if (gpio_get(PIN_CS))
            {
                state = STATE_YELLOW_BLINKING;
            } 
            watchdog_update();
            vTaskDelay(30);
        }
    }
}

void tHandleState(void* p) {
    while (true) {
        for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
            gpio_put(pins[i], 0);
        }
        state_functions[state]();
    }
}

static void state_yellow_blinking(void) {
    for (int i = 0; i < 10; i++) {
        gpio_put(pins[LIGHT_YELLOW], 1);
        vTaskDelay(DURATION_YELLOW_BLINKING);
        gpio_put(pins[LIGHT_YELLOW], 0);
        vTaskDelay(DURATION_YELLOW_BLINKING);
    }

    state = STATE_RED;
}

static void state_yellow(void) {
    gpio_put(pins[LIGHT_YELLOW], 1);

    vTaskDelay(DURATION_YELLOW);
    state = STATE_RED;
}

static void state_red(void) {
    gpio_put(pins[LIGHT_RED], 1);

    vTaskDelay(DURATION_RED);
    state = STATE_RED_YELLOW;
}

static void state_red_yellow(void) {
    gpio_put(pins[LIGHT_RED], 1);
    gpio_put(pins[LIGHT_YELLOW], 1);

    vTaskDelay(DURATION_RED_YELLOW);
    state = STATE_GREEN;
}

static void state_green(void)
{
    gpio_put(pins[LIGHT_GREEN], 1);

    vTaskDelay(DURATION_GREEN);
    state = STATE_GREEN_BLINKING;
}

static void state_green_blinking(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_put(pins[LIGHT_GREEN], 0);
        vTaskDelay(DURATION_GREEN_BLINKING);
        gpio_put(pins[LIGHT_GREEN], 1);
        vTaskDelay(DURATION_GREEN_BLINKING);
    }

    state = STATE_YELLOW;
}
