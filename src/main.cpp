#include <Arduino.h>

/**
 * The example demonstrates how to port LVGL.
 *
 * ## How to Use
 *
 * To use this example, please firstly install `ESP32_Display_Panel` (including its dependent libraries) and
 * `lvgl` (v8.3.x) libraries, then follow the steps to configure them:
 *
 * 1. [Configure ESP32_Display_Panel](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-esp32_display_panel)
 * 2. [Configure LVGL](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-lvgl)
 * 3. [Configure Board](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-board)
 *
 * ## Example Output
 *
 * ```bash
 * ...
 * Hello LVGL! V8.3.8
 * I am ESP32_Display_Panel
 * Starting LVGL task
 * Setup done
 * Loop
 * Loop
 * Loop
 * Loop
 * ...
 * ```
 */

#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <driver/twai.h>
#include "esp_task_wdt.h"

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (8 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif /* ESP_PANEL_LCD_BUS_TYPE */

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}
#endif

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        taskYIELD();
    }
}

#define MAX_STATS 30  // Maximal 30 Nachrichten für die Statistik

// Struktur zur Speicherung der Nachrichten-ID und ihrer Häufigkeit
typedef struct {
    uint32_t id;
    int count;
} CanMessageStat;

// Array zur Speicherung der Statistik
CanMessageStat messageStats[MAX_STATS];

// Buffer für die Nachrichtenanzeige
#define MAX_CAN_MESSAGES 30 // Maximale Anzahl von Nachrichten
#define MESSAGE_BUFFER_SIZE 128 // Maximale Länge pro Nachricht

// Ringpuffer für CAN-Nachrichten
char message_log[MAX_CAN_MESSAGES][MESSAGE_BUFFER_SIZE];
size_t current_message_index = 0;

bool sending = false;
uint32_t can_message_id = 0x001;
uint32_t send_interval_ms = 1000;

// Funktion zum Empfangen der CAN-Nachrichten
void receive_can_message() {
    twai_message_t message;
    char message_buffer[MESSAGE_BUFFER_SIZE];

    // Empfang der CAN-Nachrichten
    if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        // Formatiere die CAN-Nachricht
        sprintf(message_buffer, "ID: 0x%X Data: ", message.identifier);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(message_buffer + strlen(message_buffer), "0x%.2X ", message.data[i]);
        }
        strcat(message_buffer, "\n");

        // Schreibe die Nachricht in den aktuellen Index des Ringpuffers
        strncpy(message_log[current_message_index], message_buffer, MESSAGE_BUFFER_SIZE - 1);
        message_log[current_message_index][MESSAGE_BUFFER_SIZE - 1] = '\0'; // Null-terminieren

        // Aktualisiere den Index für die nächste Nachricht
        current_message_index = (current_message_index + 1) % MAX_CAN_MESSAGES;

        // Statistik aktualisieren
        bool found = false;
        for (int i = 0; i < MAX_STATS; i++) {
            if (messageStats[i].id == message.identifier) {
                messageStats[i].count++;
                found = true;
                break;
            }
        }
        if (!found) {
            // Neue Nachricht hinzufügen
            for (int i = 0; i < MAX_STATS; i++) {
                if (messageStats[i].count == 0) {
                    messageStats[i].id = message.identifier;
                    messageStats[i].count = 1;
                    break;
                }
            }
        }

        // Aktualisiere die TextArea mit allen Nachrichten
        char combined_log[MAX_CAN_MESSAGES * MESSAGE_BUFFER_SIZE] = "";
        for (size_t i = 0; i < MAX_CAN_MESSAGES; i++) {
            size_t index = (current_message_index + i) % MAX_CAN_MESSAGES; // Hole die korrekte Reihenfolge
            if (message_log[index][0] != '\0') { // Überspringe leere Einträge
                strcat(combined_log, message_log[index]);
            }
        }

        // TextArea aktualisieren
        lv_textarea_set_text(ui_TextArea1, combined_log);
    }

    // Watchdog zurücksetzen
    esp_task_wdt_reset();
}


// Timer zur regelmäßigen Aktualisierung
void update_can_log_task() {
    lv_timer_create([](lv_timer_t *timer) {
        receive_can_message();  // Nachrichten empfangen und UI aktualisieren
    }, 5000, NULL);  // Aktualisierung alle 5 Sekunden
}

void update_can_statistics(uint32_t can_id) {
    for (int i = 0; i < MAX_STATS; i++) {
        if (messageStats[i].id == can_id) {
            messageStats[i].count++;  // Wenn die ID bereits existiert, erhöhe den Zähler
            return;
        } else if (messageStats[i].id == 0) {
            messageStats[i].id = can_id;  // Neue Nachricht, füge ID und Count hinzu
            messageStats[i].count = 1;
            return;
        }
    }
}

void receive_can_message_stat() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        // Statistiken aktualisieren
        update_can_statistics(message.identifier);
    }
}

void find_top_10_messages(CanMessageStat top_10[10]) {
    // Temporäres Array kopieren
    CanMessageStat temp[MAX_STATS];
    memcpy(temp, messageStats, sizeof(messageStats));

    // Nach Häufigkeit sortieren
    for (int i = 0; i < MAX_STATS - 1; i++) {
        for (int j = i + 1; j < MAX_STATS; j++) {
            if (temp[j].count > temp[i].count) {
                CanMessageStat swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }

    // Leeren, um sicherzustellen, dass keine ungültigen Daten übernommen werden
    memset(top_10, 0, sizeof(CanMessageStat) * 10); 
    // Die Top 10 extrahieren
    for (int i = 0; i < 10; i++) {
        top_10[i] = temp[i];
    }
}

void update_x_axis_labels(lv_obj_t *chart, uint32_t IDs[10]) {
    static lv_obj_t *x_labels[10] = {NULL}; // Speicher für die Labels (persistent)

    // Berechne die Höhe des Diagramms und den Abstand zwischen den Labels
    lv_coord_t label_spacing = 20; // Fester Abstand zwischen den Labels

    for (int i = 0; i < 10; i++) {
        if (x_labels[i] == NULL) {
            // Erstmalige Erstellung der Labels
            x_labels[i] = lv_label_create(lv_obj_get_parent(chart));
            lv_obj_set_width(x_labels[i], LV_SIZE_CONTENT);
            lv_obj_set_height(x_labels[i], LV_SIZE_CONTENT);

            // Positioniere das Label links vom Diagramm
            lv_obj_align_to(x_labels[i], chart, LV_ALIGN_OUT_RIGHT_TOP, 40, i * label_spacing);
        }

        // Aktualisiere den Text des Labels
        char label_text[16];
        snprintf(label_text, sizeof(label_text), "%d: 0x%X",i+1 ,IDs[i]);
        lv_label_set_text(x_labels[i], label_text);
    }
}


void update_diagram(void) {
    // Hole die Top 10 Nachrichten
    CanMessageStat top_10[10];
    find_top_10_messages(top_10);
    uint32_t IDs[10];
    
    for (int i = 0; i < 10; i++) {
        if (top_10[i].count > 0) {
            lv_chart_set_value_by_id(ui_Chart1, ui_chart_series_1, i,top_10[i].count);
            IDs[i] = top_10[i].id;
            Serial.printf("ID: 0x%X -> Count: %d\n", top_10[i].id, top_10[i].count);
        }
    }
    
    update_x_axis_labels(ui_Chart1, IDs);
    lv_chart_refresh(ui_Chart1);

}
/*
// Logik für ID-Buttons
void id_up_handler() {
    if (can_message_id < 0x7FF) {
        can_message_id++;
        Serial.printf("CAN Message ID: 0x%X\n", can_message_id);
        if (ui_Label17 != NULL) {
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "   0x%X", can_message_id);
            lv_label_set_text(ui_Label17, buffer);
        }
    }
}

void id_down_handler() {
    if (can_message_id > 0x000) {
        can_message_id--;
        Serial.printf("CAN Message ID: 0x%X\n", can_message_id);
        if (ui_Label17 != NULL) {
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "   0x%X", can_message_id);
            lv_label_set_text(ui_Label17, buffer);
        }
    }
}

// Logik für Intervall-Buttons
void interval_up_handler() {
    if(send_interval_ms < 50000) {
        send_interval_ms += 100;
        Serial.printf("Send Interval: %d ms\n", send_interval_ms);
        if (ui_Label18 != NULL) {
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "   %d ms", send_interval_ms);
            lv_label_set_text(ui_Label18, buffer);
        }
    }
}

void interval_down_handler() {
    if (send_interval_ms > 100) {
        send_interval_ms -= 100;
        Serial.printf("Send Interval: %d ms\n", send_interval_ms);
        if (ui_Label18 != NULL) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "   %d ms", send_interval_ms);
        lv_label_set_text(ui_Label18, buffer);
        }
    }
}

// Logik für Start/Stop-Button
void send_start_handler() {
    sending = !sending;
    if (sending) {
        Serial.println("Sending started.");
    } else {
        Serial.println("Sending stopped.");
    }
}
*/

// Funktion zur Aktualisierung des Balkendiagramms
void update_chart_with_statistics() {
    // Leere das Diagramm
    lv_chart_remove_series(ui_Chart1, NULL);
    
    // Erstelle eine neue Serie im Diagramm
    lv_chart_series_t * ser = lv_chart_add_series(ui_Chart1, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    
    // Füge die Häufigkeiten der Nachrichten zum Diagramm hinzu
    for (int i = 0; i < MAX_STATS; i++) {
        if (messageStats[i].id != 0) {
            lv_chart_set_next_value(ui_Chart1, ser, messageStats[i].count);
        } else {
            break;  // Ende der Statistik-Daten erreicht
        }
    }
    
    // Aktualisiere das Diagramm
    lv_chart_refresh(ui_Chart1);

    esp_task_wdt_reset();
}

void start_chart_update_task() {
    lv_timer_create([](lv_timer_t * timer) {
        update_chart_with_statistics();  // Diagramm alle 10 Sekunden aktualisieren
    }, 15000, NULL);  // 10000 ms = 10 Sekunden
}

void simulate_can_message() {
    // Simuliere eine zufällige CAN-ID
    uint32_t fake_id = rand() % 0x7FF;  // Zufällige ID im Bereich 0x000 bis 0x7FF

    // Formatiere eine vereinfachte Nachricht
    char message_buffer[MESSAGE_BUFFER_SIZE];
    sprintf(message_buffer, "ID: 0x%X Data: TEST\n", fake_id);

    // Schreibe die Nachricht in den Ringpuffer
    strncpy(message_log[current_message_index], message_buffer, MESSAGE_BUFFER_SIZE - 1);
    message_log[current_message_index][MESSAGE_BUFFER_SIZE - 1] = '\0'; // Null-terminieren

    // Aktualisiere den Index für die nächste Nachricht
    current_message_index = (current_message_index + 1) % MAX_CAN_MESSAGES;

    // Aktualisiere die Statistik
    bool found = false;
    for (int i = 0; i < MAX_STATS; i++) {
        if (messageStats[i].id == fake_id) {
            messageStats[i].count++; // Zähler erhöhen, wenn ID gefunden
            found = true;
            break;
        }
    }
    if (!found) {
        // Neue Nachricht zur Statistik hinzufügen
        for (int i = 0; i < MAX_STATS; i++) {
            if (messageStats[i].count == 0) { // Freien Platz finden
                messageStats[i].id = fake_id;
                messageStats[i].count = 1;
                break;
            }
        }
    }

    // Kombiniere alle Nachrichten aus dem Ringpuffer für die TextArea
    char combined_log[MAX_CAN_MESSAGES * MESSAGE_BUFFER_SIZE] = "";
    for (size_t i = 0; i < MAX_CAN_MESSAGES; i++) {
        size_t index = (current_message_index + i) % MAX_CAN_MESSAGES; // Hole die korrekte Reihenfolge
        if (message_log[index][0] != '\0') { // Überspringe leere Einträge
            strncat(combined_log, message_log[index], sizeof(combined_log) - strlen(combined_log) - 1);
        }
    }

    // Aktualisiere die TextArea nur mit der neuesten Nachricht
    lv_textarea_set_text(ui_TextArea1, combined_log);
}

// Simuliere alle paar Sekunden eine Nachricht
void start_can_simulation_task() {
    lv_timer_create([](lv_timer_t * timer) {
        simulate_can_message();
    }, 5000, NULL);  // Simuliere alle 5 Sekunden eine neue Nachricht
}


void setup()
{
    Serial.begin(115200); /* prepare for possible serial debug */

    // Setze das Watchdog-Timeout auf 10 Sekunden
    esp_task_wdt_init(10, true);  // Timeout auf 10 Sekunden
    esp_task_wdt_add(NULL);       // Füge die aktuelle Task dem Watchdog hinzu

    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    /* Using double buffers is more faster than single buffer */
    /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH
    /* Initialize the input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif
    /* Initialize bus and device of panel */
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    /* Register a function to notify LVGL when the panel is ready to flush */
    /* This is useful for refreshing the screen using DMA transfers */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /**
     * These development boards require the use of an IO expander to configure the screen,
     * so it needs to be initialized in advance and registered with the panel for use.
     *
     */
    Serial.println("Initialize IO expander");
    /* Initialize IO expander */
    // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    // Turn off backlight
    // expander->digitalWrite(USB_SEL, LOW);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);
    
    ui_init();

    /* Release the mutex */
    lvgl_port_unlock();

    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    start_can_simulation_task();
    
    /* Start CAN-Bus Nachrichtenempfang */
    update_can_log_task();

    update_diagram();

    /* Release the mutex */
    lvgl_port_unlock();

    /* Start Diagramm-Aktualisierung */
    //start_chart_update_task();

    Serial.println("Setup done");
}

void loop()
{
    // Serial.println("Loop");
    //sleep(1);
    if (ui_chart_series_1 == NULL) {
        Serial.printf("Error: Chart series not initialized.\n");
    return;
    }
    if (ui_Chart1 == NULL) {
        Serial.printf("Error: Chart object not initialized.\n");
        return;
    } 
    // Füge eine kurze Verzögerung ein, um den Watchdog zurückzusetzen, aber vermeide sleep()
    vTaskDelay(pdMS_TO_TICKS(1));

    // Reset des Task-Watchdog in der loop() Funktion
    esp_task_wdt_reset();


}
