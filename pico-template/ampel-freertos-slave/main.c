#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/watchdog.h"

#define SCK 18 //sck
#define CSn 17 //cs
#define Rx 21 //miso
#define Tx 19 //mosi

#define BUFFER_SIZE 0x100
#define ERROR_CODE 0xFF

// Define the GPIO pins for the traffic lights
#define LIGHT_RED    0
#define LIGHT_YELLOW 1
#define LIGHT_GREEN  2
#define PICO_DEFAULT_LED_PIN 25

// Define the durations for each traffic light state
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

// Define the number of traffic light states
#define NUM_STATES 6

// Define the traffic light states
typedef enum {
    STATE_YELLOW_BLINKING,
    STATE_YELLOW,
    STATE_RED,
    STATE_RED_YELLOW,
    STATE_GREEN,
    STATE_GREEN_BLINKING,
} State;

typedef struct {
    State state;
    uint8_t status;
} StateStatusPair;

StateStatusPair stateStatusMap[] = {
    { STATE_YELLOW_BLINKING, STATUS_YELLOW_BLINKING },
    { STATE_YELLOW, STATUS_YELLOW },
    { STATE_RED, STATUS_RED },
    { STATE_RED_YELLOW, STATUS_RED_YELLOW },
    { STATE_GREEN, STATUS_GREEN },
    { STATE_GREEN_BLINKING, STATUS_GREEN_BLINKING }
};

TaskHandle_t tsDataTransmitter = NULL;
TaskHandle_t tsStateHandler = NULL;

// Define the function pointer type for the state functions
typedef void (*State_Function)(void);

// Define the state function prototypes
static void state_yellow_blinking(void);
static void state_yellow(void);
static void state_red(void);
static void state_red_yellow(void);
static void state_green(void);
static void state_green_blinking(void);

// Define the state function array
static State_Function state_functions[NUM_STATES] = {
    state_yellow_blinking,
    state_yellow,
    state_red,
    state_red_yellow,
    state_green,
    state_green_blinking,
};

// Define the traffic light pin array
static const int pins[] = {
    LIGHT_RED,
    LIGHT_YELLOW,
    LIGHT_GREEN,
};

uint8_t rx_buffer[BUFFER_SIZE] = {0};

uint8_t getStatusForState(State state) {
    for (int i = 0; i < NUM_STATES; i++) {
        if (stateStatusMap[i].state == state) {
            return stateStatusMap[i].status;
        }
    }
    // If no matching state was found, return an error value (-1)
    return -1;
}

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

// Set the initial traffic light state
State state = STATE_YELLOW_BLINKING;

void transmit_state(void* p) {
    for( ;; ) {
        // Select the slave by setting its CS pin low
        gpio_put(CSn, 0);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        // Transmit data
        uint8_t send_buf = getStatusForState(state);
        if (spi_is_writable(spi0))
        {
            spi_write_read_blocking(spi0, &send_buf, rx_buffer, sizeof(send_buf));
        } else {
            vTaskDelete(tsStateHandler);
            break;
        }
        

        // Deselect the slave by setting its CS pin high
        gpio_put(CSn, 1);

        vTaskDelay(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(10);

        //wenn fehler zurückkommt restarte.
        if (rx_buffer[0] != ERROR_CODE)
        {
            watchdog_update();
        }
        
    }

    //error mode
    while (1)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        gpio_put(pins[LIGHT_YELLOW], 1);
        sleep_ms(DURATION_YELLOW_BLINKING);
        gpio_put(pins[LIGHT_YELLOW], 0);
        sleep_ms(DURATION_YELLOW_BLINKING);
    }
    
}

void handle_state(void* p) {
    for ( ;; ) {
        // Turn off all traffic lights
        for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
            gpio_put(pins[i], 0);
        }

        // Call the current state function
        state_functions[state]();
    }
}

int main() {
    // Initialize the GPIO pins
    init_pins();
    init_spi();

    xTaskCreate(transmit_state, "dataTransmitter", 1024, NULL, 1, &tsDataTransmitter);
    xTaskCreate(handle_state, "stateHandler", 1024, NULL, 1, &tsStateHandler);
    vTaskStartScheduler();

    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(60, 1);

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        gpio_put(pins[LIGHT_YELLOW], 1);
        sleep_ms(DURATION_YELLOW_BLINKING);
        gpio_put(pins[LIGHT_YELLOW], 0);
        sleep_ms(DURATION_YELLOW_BLINKING);
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
