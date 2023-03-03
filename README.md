# Time Critical Datatransmission

## Fragestellungen

- Was ist ein SPI-Bus und wie ist dieser aufgebaut?
- Welche Vorteile ergeben sich bei der Verwendung eines Kommunikationsbusses?
- Welche Möglichkeiten der Beschaltung sind beim SPI-Bus möglich und wie wirkt sich die Clock darauf aus?
- Wie werden zeitkritische Anwendungen (real-time) eingeteilt?
- Wie kommt ein Watchdog bei zeitkritischen Anwendungen zum Einsatz?
- Wie kann man Interrupts priorisieren?
- Was sind Real-Time Operating-Systems (RTOS) und wie kann man diese auf Mikrokontrollern einsetzen?

## Implementierung

### Setup

Für die Übung wurden folgende Komponenten benötigt:

- 2x Raspberry Pico

- 1x Rote LED

- 1x Gelbe LED

- 1x Grüne LED

- 1x 330 Ohm Widerstand

- 4x Männlich-Männlich Kabel

- 2x Micro-USB Kabel

- 1x Logic Analyzer

- 4x Männlich-Weiblich Kabel

- 1x Steckbrett



**Verkabelung:**

Für einen SPI Bus benötigt man vier Verbindungen. [3]

1. MISO (RX)

2. MOSI (TX)

3. SC (SCn)

4. SCK (SCKL)



Der erste Schritt ist daher am PinOut [1] die notwendigen Anschlüsse/Pins zu identifizieren und die beiden PICO's miteinander zu verkabeln. In meinem Fall habe ich folgende Pins gewählt:

| PICO 1 (Slave)  | PICO 2 (Master) |
| --------------- | --------------- |
| GP18 (SPI0 SCK) | GP2 (SPI0 SCK)  |
| GP19 (SPI0 TX)  | GP3 (SPI0 TX)   |
| GP16 (SPI0 RX)  | GP4 (SPI0 RX)   |
| GP17 (SPI0 CSn) | GP5 (SPI0 CSn)  |

Wichtig zu beachten ist hier, dass man immer den selben SPI (in meinem Fall SPI0) wählt. 



Da wir eine Ampelschaltung bauen möchten, müssen wir natürlich auch noch die LED's richtig am Slave installieren. Da LED's einen Minus und einen Plus Pol (Anode und Kathode) haben, gilt es zunächst einmal herauszufinden was Anode und was Kathode ist. [2]

Anschließend benötigt man lediglich drei GPIO Ports (in meinem Fall GP0, GP1 und GP2) und schließt die Anode an diese Ports an. 

Anschließend benötigt man nur noch einen Widerstand, damit die LED's nicht kaputt gehen, auch diesen schließt man an einen beliebigen GND Port an. 

![](/Users/pelias/Documents/10.SEM/SYT/BORM/syt5-gek1011-timecritical-datatransmission-pdamianik-pelias-1/images/slave.png)

![](/Users/pelias/Documents/10.SEM/SYT/BORM/syt5-gek1011-timecritical-datatransmission-pdamianik-pelias-1/images/master.png)

![](/Users/pelias/Documents/10.SEM/SYT/BORM/syt5-gek1011-timecritical-datatransmission-pdamianik-pelias-1/images/slave-master.png)

**Software**

Um die PICO's programmieren zu können benötigen wir eine angemessene Entwicklungsumgebung. Daher ist es ratsam das `pico-template` [4] von Prof. Michael Borko zu clonen und die Installationsschritte darin zu befolgen.

Funktioniert alles wie beschrieben ist man mit dem Setup fertig und kann an die Entwicklung gehen.



**Logic Analyzer**

tba



### Entwicklung

#### Slave

Bevor man beginnt sollte man sicherstellen, dass man weiß wie eine Ampel funktioniert. Sowohl im normal Zustand, als auch im Fehlerfall oder Startvorgang. [5]

Hat man das verstanden, kann man beginnen zu entwickeln. 

Im pico-template erstellt man einen neuen Ordner (`ampel-freertos-slave`) und in diesem zwei Files (`main.c`, `CMakeLists.txt`). Das CMakeLists kann aus den anderen Beispielen im Template kopiert werden. Wichtig ist nur, dass die richtigen Libraries gelinkt werden:

```cmake
target_link_libraries(ampel-freertos-slave pico_stdlib hardware_spi pico_multicore FreeRTOS)
```

Im `main.c` kommt der eigentlich wichtige Teil unsere Software hin und ist daher auch wesentlich umfangreicher.

Im Ersten Schritt sollten wir uns darum kümmern, dass unsere Ampel mal funktioniert, ob etwas über den SPI Bus verschickt wird ist uns da mal egal.

**Ampel**

Wir wollen unsere Ampelschaltung möglichst schön implementieren, daher benutzen wir Funktionen, die über einen Pointer übergeben werden und ihren State setzen => StateManagement.

Daher definieren wir unsere States und die Anzahl:

```c

#define NUM_STATES 6

typedef enum {
    STATE_YELLOW_BLINKING,
    STATE_YELLOW,
    STATE_RED,
    STATE_RED_YELLOW,
    STATE_GREEN,
    STATE_GREEN_BLINKING,
} State;

```

Außerdem braucht es einen default Zustand:

```c
State state = STATE_YELLOW_BLINKING;
```

Des weiteren brauchen wir unsere LED Pins, da wir diese ja Ein- und Ausschalten möchten:

```c
#define LIGHT_RED    0
#define LIGHT_YELLOW 1
#define LIGHT_GREEN  2
#define PICO_DEFAULT_LED_PIN 25


static const int pins[] = {
    LIGHT_RED,
    LIGHT_YELLOW,
    LIGHT_GREEN,
};
```

Der PICO_DEFAULT_LED_PIN ist die LED die direkt am PICO installiert ist.

Als nächstes Überlegen wir uns, wie lange jeder Zustand der Ampel hält, bzw. was dieser Zustand für Timings hat. Diese definieren wir dann ebenfalls:

```c
#define DURATION_YELLOW_BLINKING 500
#define DURATION_YELLOW          4000
#define DURATION_RED             5000
#define DURATION_RED_YELLOW      4000
#define DURATION_GREEN           5000
#define DURATION_GREEN_BLINKING  500
```

Nun haben wir alle notwendigen Informationen initialisiert und können mit dem Code beginnen.

Das wichtigste zuerst: Die Pins initialisieren, sodass wir sie verwenden können. Dafür haben wir eine eigene Methode erstellt:

```c
static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
    }
}
```

Nun können wir uns daran setzen, die einzelnen Funktionen für die verschiedenen Ampelstates zu erstellen. (ACHTUNG: `vTaskDelay` ist bereits eine freeRTOS methode und sollte mit `sleep_ms` ersetzt werden sofern man kein freeRTOS verwendet.)

```c
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
```

Dann noch die Referenzen speichern:

```c
typedef void (*State_Function)(void);

static void state_yellow_blinking(void);
static void state_yellow(void);
static void state_red(void);
static void state_red_yellow(void);
static void state_green(void);
static void state_green_blinking(void);

static State_Function state_functions[NUM_STATES] = {
    state_yellow_blinking,
    state_yellow,
    state_red,
    state_red_yellow,
    state_green,
    state_green_blinking,
};
```

Die Ampel sollte nun bereits funktionieren, alles was fehlt ist die Main Methode. Jedoch möchten wir regelmäßig den aktuellen Status über den SPI Bus verschicken, weswegen wir zu unserem nächsten Schritt kommen.

**SPI**

Für den SPI Bus müssen wir zunächst wieder einige Variablen instanzieren.

```c
#define PIN_SCK 18 //sck
#define PIN_CS 17 //cs
#define PIN_RX 21 //miso
#define PIN_TX 19 //mosi
```

Anschließend müssen wir den SPI Bus wiederum initialisieren, wofür wir auch eine eigene Funktion erstellt haben. Besonders wichtig ist hier, dass der SPI Mode auf slave gesetzt wird:

```c
static void init_spi(void) {
    spi_init(spi0, 1000 * 1000); 
    spi_set_slave(spi0, true);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
} 
```

Theoretisch könnte man nun bereits über den SPI Bus senden, jedoch haben wir aktuell ein enormes Problem: Die Task delays würden ein senden über den SPI Bus immer wieder verzögern oder verhindern. Daher benötigen wir mehrere Tasks die Asynchron laufen können. 

**Watchdog / FreeRTOS**

Hier müssen wir zunächst auch wieder einige Definitionen erstellen. Aus der Aufgabenstellung entnehmen wir folgende Codes für die einzelnen Stati:

| Status        | Code    |
| ------------- | ------- |
| Rot           | 1-1-1-0 |
| Rot-Gelb      | 1-1-0-1 |
| Grün          | 0-0-1-0 |
| Grün blinkend | 0-1-0-1 |
| Gelb          | 1-0-0-0 |
| Gelb blinkend | 0-0-0-1 |

Diese haben wir in Hex Code codiert:

```c
#define STATUS_RED            0xE
#define STATUS_RED_YELLOW     0xD
#define STATUS_GREEN          0x2
#define STATUS_GREEN_BLINKING 0x5
#define STATUS_YELLOW          0x8
#define STATUS_YELLOW_BLINKING 0x1
#define ERROR_CODE 0xFF
```

Des weiteren benötigen wir eine Hashmap die jeden dieser Codes einem Status zuordnet:

```c
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
    { STATE_GREEN_BLINKING, STATUS_GREEN_BLINKING }
};

static uint8_t getStatusForState(State state) {
    for (int i = 0; i < NUM_STATES; i++) {
        if (stateStatusMap[i].state == state) {
            return stateStatusMap[i].status;
        }
    }
    return -1;
}
```

Und weil wir zwei Tasks benötigen definieren wir auch gleich die beiden Task Variablen:

```c
TaskHandle_t tsDataTransmitter = NULL;
TaskHandle_t tsStateHandler = NULL;
```

Danach erstellen wir die StateManager Funktion:

```c
void handle_state(void* p) {
    while (true) {
        for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
            gpio_put(pins[i], 0);
        }
        state_functions[state]();
    }
}
```

Das Daten versenden ist dann wesentlich komplexer, hier müssen wir nämlich nicht nur die Daten über den SPI Bus verschicken, sondern auch darauf achten, was zurückkommt, den watchdog aktualisieren und im Fehlerfall in den Fehlermode gehen und die Ampel herunterfahren.

```c

#define ERROR_CODE 0xFF
void transmit_state(void* p) {
    uint8_t rx_buffer[10];
    while (true) {
        gpio_put(PIN_CS, 0);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        uint8_t send_buf = getStatusForState(state);
        if (spi_is_writable(spi0))
        {
            spi_write_read_blocking(spi0, &send_buf, rx_buffer, sizeof(send_buf));
        } else {
            vTaskDelete(tsStateHandler);
            break;
        }
        
        gpio_put(PIN_CS, 1);

        vTaskDelay(10);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(10);

        if (rx_buffer[0] != ERROR_CODE)
        {
            watchdog_update();
        }
    }

    while (1)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        gpio_put(pins[LIGHT_YELLOW], 1);
        sleep_ms(DURATION_YELLOW_BLINKING);
        gpio_put(pins[LIGHT_YELLOW], 0);
        sleep_ms(DURATION_YELLOW_BLINKING);
    }
    
}
```

Das einzige was uns jetzt noch Fehlt ist die Main Methode. Diese ist ziemlich simpel:

```c
int main() {
    init_pins();
    init_spi();

    xTaskCreate(transmit_state, "dataTransmitter", 1024, NULL, 1, &tsDataTransmitter);
    xTaskCreate(handle_state, "stateHandler", 1024, NULL, 1, &tsStateHandler);
    vTaskStartScheduler();
    
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
```



#### Master

Der Master ist dafür da, um sicherzustellen, dass der Slave die Ampel richtig besteuert und alles funktioniert.  

Das Grundlegende Setup ist ident wie beim Slave.

**SPI**

Auch hier müssen zunächst die Pins definiert werden:

```c
#define PIN_SCK 2 //sck
#define PIN_CS 5 //cs
#define PIN_RX 4 //miso
#define PIN_TX 3 //mosi
```

Anschließend werden sie mit fast der selben Funktion wie beim slave initialisiert. (Achtung: Slave Modus wird nun deaktiviert -> ist ja der Master)

```c
static void init_spi(void) {
    spi_init(spi0, 1000 * 1000);
    spi_set_slave(spi0, false);
    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
} 
```

**LED**

Auch beim Master PICO verwenden wir die Default LED, daher müssen wir folgendes definieren:

```c
#define ERROR_CODE 0xFF
#define PICO_DEFAULT_LED_PIN 25
```

und die initalisierungs Funktion:

```c
static void init_pins(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}
```

**FreeRTOS**

Zusätzlich zu zwei Tasks definieren wir eine Variable für den Timestamp:

```c
TaskHandle_t tsDataHandler = NULL;
TaskHandle_t tsBlinker = NULL;
TickType_t timestamp = 0;
```

Dann einen `blinker` Task, welcher lediglich dafür sorgt, dass die LED blinkt:

```c
void tBlinker(void* p) {
    while (true) {
        vTaskDelay(30);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(30);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}
```

Und einen etwas komplexeren DataHandler Task:

```c
void tDataHandler(void* p) {
    uint8_t buffer[10];
    while (true) {
        gpio_put(PIN_CS, 0);
        if (spi_is_readable(spi0))
        {
            timestamp = xTaskGetTickCount();
            spi_read_blocking(spi0, 0, buffer, sizeof(buffer));
        }
        
        if ((xTaskGetTickCount() - timestamp) > 60)
        {
            if (spi_is_writable(spi0))
            {
                uint8_t send[] = {ERROR_CODE};
                spi_write_blocking(spi0, send, sizeof(send));
            }
        }
        gpio_put(PIN_CS, 1);
        vTaskDelay(30);
    }
}
```

Dieser speichert vor dem empfangen die aktuelle Zeit um später zu kontrollieren ob der Slave innerhalb von 60ms einen State transferiert hat. Wenn nicht wird ein ErrorCode verschickt.

Was nun noch fehlt ist die `main` Methode:

```c
int main() {
    init_pins();
    init_spi();

    xTaskCreate(tDataHandler, "dataHandler", 1024, NULL, 1, &tsDataHandler);
    xTaskCreate(tBlinker, "blinkHandler", 1024, NULL, 1, &tsBlinker);
    vTaskStartScheduler();

    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }
}
```



## Quellen

[1] "Raspberry Pi Pico Pinout"; "raspberrypi.com"; [Link](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf); zuletzt besucht am 03.03.2023

[2] "IDENTIFY THE ANODE/CATHODE of LED's"; "instructables.com"; [Link](https://www.instructables.com/IDENTIFY-THE-ANODECATHODE-of-LEDs/); zuletzt besucht am 03.03.2023

[3] "Serial Peripheral Interface"; "mikrocontroller.net"; [Link](https://www.mikrocontroller.net/articles/Serial_Peripheral_Interface); zuletzt besucht am 03.03.2023

[4] "Raspberry Pi RP2040 Template"; "Michael Borko"; [Link](https://github.com/mborko/pico-template); zuletzt besucht am 03.03.2023

[5] "Kleine Ampelkunde - Lichtzeichen"; "wien.gv.at"; [Link](https://www.wien.gv.at/verkehr/ampeln/ampelkunde.html); zuletzt besucht am 03.03.2023
