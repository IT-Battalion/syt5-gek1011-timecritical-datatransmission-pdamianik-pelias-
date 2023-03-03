#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SCK 18 //sck
#define CSn 17 //cs
#define Rx 21 //miso
#define Tx 19 //mosi
#define BUFFER_SIZE 10

// Define the GPIO pins for the traffic lights
#define LIGHT_RED    0
#define LIGHT_YELLOW 1
#define LIGHT_GREEN  2

// Define the durations for each traffic light state
#define DURATION_YELLOW_BLINKING 500
#define DURATION_YELLOW          4000
#define DURATION_RED             5000
#define DURATION_RED_YELLOW      4000
#define DURATION_GREEN           5000
#define DURATION_GREEN_BLINKING  500

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

uint8_t tx_buffer[BUFFER_SIZE] = {0};
uint8_t rx_buffer[BUFFER_SIZE] = {0};

// Initialize the traffic light pins and direction
static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}

static void init_spi(void) {
    // Initialize SPI interface
    spi_init(spi_default, 100 * 1000 * 1000);  // Set clock frequency to 100 MHz
    spi_set_slave(spi_default, true);
    gpio_set_function(Rx, GPIO_FUNC_SPI);
    gpio_set_function(CSn, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(Tx, GPIO_FUNC_SPI);

    gpio_set_dir(CSn, GPIO_OUT);

    // Make the SPI pins available to picotool
    //bi_decl(bi_4pins_with_func(Rx, Tx, SCK, CSn, GPIO_FUNC_SPI));
} 

static void transmit_state(State state) {
    // Select the slave by setting its CS pin low
    gpio_put(CSn, 0);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    // Transmit data
    spi_write_read_blocking(spi0, tx_buffer, rx_buffer, BUFFER_SIZE);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // Deselect the slave by setting its CS pin high
    gpio_put(CSn, 1);
}

// Set the initial traffic light state
State state = STATE_YELLOW_BLINKING;

int main() {
    // Initialize the GPIO pins
    init_pins();
    init_spi();

    // Run the traffic light state machine loop
    for (;;) {
        // Turn off all traffic lights
        for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
            gpio_put(pins[i], 0);
        }

        // Call the current state function
        state_functions[state]();
        transmit_state(state);
    }
}

static void state_yellow_blinking(void) {
    for (int i = 0; i < 10; i++) {
        gpio_put(pins[LIGHT_YELLOW], 1);
        sleep_ms(DURATION_YELLOW_BLINKING);
        gpio_put(pins[LIGHT_YELLOW], 0);
        sleep_ms(DURATION_YELLOW_BLINKING);
    }

    state = STATE_RED;
}

static void state_yellow(void) {
    gpio_put(pins[LIGHT_YELLOW], 1);

    sleep_ms(DURATION_YELLOW);
    state = STATE_RED;
}

static void state_red(void) {
    gpio_put(pins[LIGHT_RED], 1);

    sleep_ms(DURATION_RED);
    state = STATE_RED_YELLOW;
}

static void state_red_yellow(void) {
    gpio_put(pins[LIGHT_RED], 1);
    gpio_put(pins[LIGHT_YELLOW], 1);

    sleep_ms(4000);
    state = STATE_GREEN;
}

void state_green(void)
{
    gpio_put(pins[LIGHT_GREEN], 1);

    sleep_ms(5000);
    state = STATE_GREEN_BLINKING;
}

void state_green_blinking(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_put(pins[LIGHT_GREEN], 0);
        sleep_ms(500);
        gpio_put(pins[LIGHT_GREEN], 1);
        sleep_ms(500);
    }

    state = STATE_YELLOW;
}
