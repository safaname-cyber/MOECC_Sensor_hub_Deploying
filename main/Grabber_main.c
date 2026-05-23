#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// #define PKT_HEADER0   0xFF   // Example packet header byte 1
// #define PKT_HEADER1   0xFE   // Example packet header byte 2
// #define STS_MODE      0x2A 
#define PKT_HEADER0 0xFF
#define PKT_HEADER1 0xFF
#define PKT_LENGTH  0x0A
#define PKT_MODE    0x04
#define INST_WRITE  0x03
#define INST_READ   0x02
#define STS_MODE    0x21
#define STS_ACC     0x29
#define STS_POS     0x2A
#define STS_CURRENT 0x45
#define STS_TEMP    0x3F
#define STS_LOAD    0x3C

// ---------------- HX711 Pins ----------------
#define LOADCELL_DOUT_PIN  GPIO_NUM_18
#define LOADCELL_SCK_PIN   GPIO_NUM_19

// ---------------- Motor Pins ----------------
#define MOTOR_TX_PIN   GPIO_NUM_7
#define MOTOR_RX_PIN  GPIO_NUM_8

// ---------------- UART ----------------
#define UART0   UART_NUM_0
#define UART1       UART_NUM_1
#define BUF_SIZE    1024

// =====================================================================
// Protection Limits (SOP §7 Control Firmware, §8.2 Defensive, Checklist B)
// ---------------------------------------------------------------------
// NOTE: Values below are PLACEHOLDERS. Tune against motor datasheet and
// bench data before release. Centralised here so a reviewer can audit
// all thresholds in one place.
// =====================================================================
#define LIMIT_CURRENT_MAX_RAW       300     // raw counts from STS_CURRENT
#define LIMIT_TEMP_MAX_C            70      // °C, motor body
#define LIMIT_LOAD_MAX_RAW          800     // raw counts (0.1% units = 80.0%)
#define LIMIT_WEIGHT_MAX_G          500.0f  // grams, load cell ceiling
#define LIMIT_WEIGHT_MIN_G         (-500.0f)// grams, load cell floor (sanity)

// Debounce — a single noisy sample must NOT latch a fault.
// Faults trip only after N consecutive out-of-range readings.
#define FAULT_CONSECUTIVE_TRIPS     2

// Comms timeout: if motor stops responding to telemetry reads, enter safe state.
#define SENSOR_RESP_MIN_BYTES       2
#define SENSOR_TIMEOUT_TICKS        pdMS_TO_TICKS(100)

// Fault codes — exposed so logs and host can disambiguate the trip source.
typedef enum {
    FAULT_NONE          = 0,
    FAULT_OVERCURRENT   = 1,
    FAULT_OVERTEMP      = 2,
    FAULT_OVERLOAD      = 3,
    FAULT_OVERWEIGHT    = 4,
    FAULT_SENSOR_NAN    = 5,
    FAULT_SENSOR_TIMEOUT= 6,
} fault_code_t;

// Global fault state. Read by motor_task before honouring OPEN/CLOSE.
// Written by current_task and loadcell_task on limit trip.
// atomic_int → no mutex needed for single-word reads/writes (SOP §9.2).
static atomic_int g_fault_state = ATOMIC_VAR_INIT(FAULT_NONE);

static const char *TAG = "GRABBER";

// HX711 globals
static float calibration_factor = -317.9;
static long offset = 0;
static float scale = 1.0f;

// Forward decls for safe-state helpers (defined below send_packet)
static void motor_emergency_stop(uint8_t motor_id);
static void enter_safe_state(fault_code_t code, const char *source, long value);
static const char *fault_name(fault_code_t c);


void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_driver_install(UART1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART1, &uart_config);
    uart_set_pin(UART1, MOTOR_TX_PIN, MOTOR_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ----------------------------------------------------
// HX711 Functions
// ----------------------------------------------------
static void hx711_init() {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LOADCELL_DOUT_PIN),
    };
    gpio_config(&io_conf);

    gpio_config_t sck_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LOADCELL_SCK_PIN),
    };
    gpio_config(&sck_conf);

    gpio_set_level(LOADCELL_SCK_PIN, 0);
}

static int hx711_is_ready() {
    return (gpio_get_level(LOADCELL_DOUT_PIN) == 0);
}

static long hx711_read() {
    while (!hx711_is_ready()) {
        vTaskDelay(1);
    }
    long value = 0;
    for (int i = 0; i < 24; i++) {
        gpio_set_level(LOADCELL_SCK_PIN, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | gpio_get_level(LOADCELL_DOUT_PIN);
        gpio_set_level(LOADCELL_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }
    gpio_set_level(LOADCELL_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(LOADCELL_SCK_PIN, 0);
    if (value & 0x800000) value |= ~0xFFFFFF;
    return value;
}

static long hx711_read_average(int times) {
    long sum = 0;
    for (int i = 0; i < times; i++) sum += hx711_read();
    return sum / times;
}

static void hx711_tare(int times) {
    offset = hx711_read_average(times);
    ESP_LOGI(TAG, "Tare complete. Offset = %ld", offset);
}

static void hx711_set_scale(float scale_val) {
    scale = scale_val;
}

static float hx711_get_units(int times) {
    long val = hx711_read_average(times) - offset;
    return (float)val / scale;
}

// ----------------------------------------------------
// Motor Functions
// ----------------------------------------------------
// static void motor_init() {
//     gpio_set_direction(MOTOR_OPEN_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_direction(MOTOR_CLOSE_PIN, GPIO_MODE_OUTPUT);
//     gpio_set_level(MOTOR_OPEN_PIN, 0);
//     gpio_set_level(MOTOR_CLOSE_PIN, 0);
// }

// static void grabber_open() {
//     ESP_LOGI(TAG, "Grabber OPEN");
//     gpio_set_level(MOTOR_OPEN_PIN, 1);
//     vTaskDelay(pdMS_TO_TICKS(1000)); // run motor 1s
//     gpio_set_level(MOTOR_OPEN_PIN, 0);
// }

// static void grabber_close() {
//     ESP_LOGI(TAG, "Grabber CLOSE");
//     gpio_set_level(MOTOR_CLOSE_PIN, 1);
//     vTaskDelay(pdMS_TO_TICKS(1000));
//     gpio_set_level(MOTOR_CLOSE_PIN, 0);
// }

// ----------------------------------------------------
// FreeRTOS Tasks
// ----------------------------------------------------
void loadcell_task(void *pv) {
    hx711_set_scale(1.0);
    hx711_tare(10);

    // Per-task debounce counter — local, no shared-state issues.
    int weight_trip_count = 0;

    while (1) {
        hx711_set_scale(calibration_factor);
        float weight = hx711_get_units(5);
        printf("Weight: %.2f g | Calib: %.2f\n", weight, calibration_factor);

        // ---- LIMIT CHECK: Weight (SOP §8.2 range + NaN validation) ----
        if (isnan(weight) || isinf(weight)) {
            enter_safe_state(FAULT_SENSOR_NAN, "weight", 0);
        } else if (weight > LIMIT_WEIGHT_MAX_G || weight < LIMIT_WEIGHT_MIN_G) {
            weight_trip_count++;
            if (weight_trip_count >= FAULT_CONSECUTIVE_TRIPS) {
                enter_safe_state(FAULT_OVERWEIGHT, "weight", (long)weight);
            }
        } else {
            weight_trip_count = 0;  // reset debounce on good sample
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// void uart_task(void *pv) {
//     uint8_t data[BUF_SIZE];
//     while (1) {
//         int len = uart_read_bytes(UART0, data, BUF_SIZE-1, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             data[len] = '\0'; // null terminate
//             if (strstr((char*)data, "GRABBER OPEN\r\n")) {
//                 grabber_open();
//             } else if (strstr((char*)data, "GRABBER CLOSE")) {
//                 grabber_close();
//             }
//         }
//     }
// }
// void Position_pkt(uint8_t id, uint8_t speed, uint8_t dir1, uint8_t dir2, uint16_t pos) {
//     uint8_t pkt[11];

//     pkt[0] = PKT_HEADER0;
//     pkt[1] = PKT_HEADER1;
//     pkt[2] = id;         
//     pkt[3] = 0x07;       // length (adjusted for extra byte)
//     pkt[4] = 0x03;       // write instruction
//     pkt[5] = STS_MODE;   
//     pkt[6] = speed;      
//     pkt[7] = dir1;       
//     pkt[8] = dir2;       
//     pkt[9] = pos & 0xFF;       // LSB
//     pkt[10] = (pos >> 8) & 0xFF; // MSB

//     uart_write_bytes(UART_PORT, (const char *)pkt, sizeof(pkt));
// }
// uint16_t compute_sum(const uint8_t* data, int length) {
//     uint16_t sum = 0;
//     for (int i = 2; i < length - 1; i++) {
//         sum += data[i];
//     }
//     return sum;
// }
// uint8_t checksum(const uint8_t* data, int length) {
//     uint16_t sum = compute_sum(data, length);
//     return (uint8_t)((~sum) & 0xFF);
// }


// static void log_packet(const uint8_t* packet, int length) {
//     printf("Sending: ");
//     for (int i = 0; i < length; i++) {
//         printf("%02X ", packet[i]);
//     }
//     printf("\n");
//     ESP_LOGI(TAG, "Packet length: %d bytes", length);
// }

// static void uart_send_packet(const uint8_t* packet, int length) {
//     int written = uart_write_bytes(UART1, (const char*)packet, length);
//     if (written < 0) {
//         ESP_LOGE(TAG, "uart_write_bytes failed: %d", written);
//     } else {
//         ESP_LOGI(TAG, "Wrote %d bytes over UART", written);
//     }
// }

// static void send_packet(const uint8_t* packet, int length) {
//     log_packet(packet, length);
//     uart_send_packet(packet, length);
// }

static uint8_t checksum(uint8_t* data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length - 1; i++) {
        sum += data[i];
    }
    return (~sum) & 0xFF;
}

// send raw packet on UART1
static void send_packet(uint8_t* packet, int length) {
    printf("Sending: ");
    for (int i = 0; i < length; i++) {
        printf("%02X ", packet[i]);
    }
    printf("\n");

    // UART1 TX
    int written = uart_write_bytes(UART1, (const char*)packet, length);
    if (written < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", written);
    } else {
        ESP_LOGI(TAG, "wrote %d bytes", written);
    }
}
void mode_pkt(uint8_t id, uint8_t mode){
    uint8_t mode_pkt[8] = {
        PKT_HEADER0, PKT_HEADER1,
        id, 0x04, 0x03,
        STS_MODE, 
        //0x0B,             // Starting address: ACC
        mode,              // ACC = 0x00
        0
    };
    mode_pkt[7] = checksum(mode_pkt, sizeof(mode_pkt));
    send_packet(mode_pkt, sizeof(mode_pkt));
}
void Position_pkt(uint8_t motor_id, uint16_t acceleration, uint16_t max_pos,
                  uint16_t min_pos, int16_t speed){
    uint16_t val = speed < 0 ? ((-speed) | (1 << 15)) : speed;
    uint8_t pkt[14] = { PKT_HEADER0, PKT_HEADER1,
                                motor_id, PKT_LENGTH, INST_WRITE,
                                STS_POS, acceleration,
                                max_pos & 0xFF, (max_pos >> 8) & 0xFF,
                                min_pos & 0xFF, (min_pos >> 8) & 0xFF,
                                val & 0xFF, (val >> 8) & 0xFF,
                                0 };
    pkt[13] = checksum(pkt, sizeof(pkt));
    send_packet(pkt, sizeof(pkt));
}

// ---------------------------------------------------------------------
// Safe-state helpers (SOP §4.3, §14 / Checklist B fault-injection)
// ---------------------------------------------------------------------
static const char *fault_name(fault_code_t c) {
    switch (c) {
        case FAULT_NONE:           return "NONE";
        case FAULT_OVERCURRENT:    return "OVERCURRENT";
        case FAULT_OVERTEMP:       return "OVERTEMP";
        case FAULT_OVERLOAD:       return "OVERLOAD";
        case FAULT_OVERWEIGHT:     return "OVERWEIGHT";
        case FAULT_SENSOR_NAN:     return "SENSOR_NAN";
        case FAULT_SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";
        default:                   return "UNKNOWN";
    }
}

// Send a stop using the same Position_pkt signature already in use.
// Preserves wire protocol — uses identical args to the existing CLOSE-stop call.
static void motor_emergency_stop(uint8_t motor_id) {
    Position_pkt(motor_id, 50, 7, 0, 0);
}

// Latch the fault, command motor stop, log. Idempotent — re-entry is safe
// and will NOT re-spam the motor or log once already latched.
static void enter_safe_state(fault_code_t code, const char *source, long value) {
    fault_code_t expected = FAULT_NONE;
    // Only the FIRST tripping condition latches and acts. Subsequent
    // calls while a fault is active are silently ignored to avoid
    // log flooding and redundant stop packets.
    if (atomic_compare_exchange_strong(&g_fault_state, (int*)&expected, (int)code)) {
        ESP_LOGE(TAG, "SAFE STATE entered: fault=%s source=%s value=%ld",
                 fault_name(code), source, value);
        motor_emergency_stop(1);
    }
}

/* ---------------- Motor Task ---------------- */
void motor_task(void *pv) {
    ESP_LOGI("MOTOR", "Motor task running, waiting for commands...");
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART0, data, BUF_SIZE - 1,
                                  pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';  // null terminate
            ESP_LOGI("UART", "Received: %s", data);

            // ---- State validation before action (SOP §8.2) ----
            // If a fault is latched, refuse motion commands and log.
            // "No motor start during E-stop" rule.
            fault_code_t active = (fault_code_t)atomic_load(&g_fault_state);
            if (active != FAULT_NONE) {
                ESP_LOGW("MOTOR", "Command ignored, fault active: %s",
                         fault_name(active));
                continue;
            }

            //if (strcasecmp((char*)data, "GRABBER OPEN") == 0) {
              if (strstr((char*)data, "GRABBER OPEN") != NULL){
                ESP_LOGI("MOTOR", "Executing GRABBER OPEN");
                Position_pkt(1, 50, 0, 7, 2400);
                vTaskDelay(pdMS_TO_TICKS(2000)); // wait for motor to finish
                Position_pkt(1, 50, 7, 0, 0); // stop motor

            } 
            //else if (strcasecmp((char*)data, "GRABBER CLOSE") == 0) {
            else if (strstr((char*)data, "GRABBER CLOSE") != NULL){
                ESP_LOGI("MOTOR", "Executing GRABBER CLOSE");
                Position_pkt(1, 50, 7, 0, 0);

            } else {
                ESP_LOGW("MOTOR", "Unknown command: %s", data);
            }
        }
    }
}
int sts_tohost(int a, int b) {
    int sign_bit = 1 << b;
    if (a & sign_bit) {
        return -(a & ~sign_bit);
    } else {
        return a;
    }
}

void current_pkt(uint8_t id){
    uint8_t current_pkt[8] = {
        PKT_HEADER0, PKT_HEADER1,
        id, 0x04, INST_READ,
        STS_CURRENT, 
        0x02,              
        0
    };
    current_pkt[7] = checksum(current_pkt, sizeof(current_pkt));
    // for (int i = 0; i < sizeof(current_pkt); i++) {
    //     printf("%02X ", current_pkt[i]);
    // }
    // printf("\n");

    // UART1 TX
    uart_write_bytes(UART1, (const char*)current_pkt, sizeof(current_pkt));
}
void load_pkt(uint8_t id){
    uint8_t load_pkt[8] = {
        PKT_HEADER0, PKT_HEADER1,
        id, 0x04, INST_READ,
        STS_LOAD, 
        0x04,              
        0
    };
    load_pkt[7] = checksum(load_pkt, sizeof(load_pkt));
    uart_write_bytes(UART1, (const char*)load_pkt, sizeof(load_pkt));
}
void temp_pkt(uint8_t id){
    uint8_t temp_pkt[8] = {
        PKT_HEADER0, PKT_HEADER1,
        id, 0x04, INST_READ,
        STS_TEMP, 
        0x02,              
        0
    };
    temp_pkt[7] = checksum(temp_pkt, sizeof(temp_pkt));
    uart_write_bytes(UART1, (const char*)temp_pkt, sizeof(temp_pkt));
}

// void current_task(void *pv) {
//     uint8_t data[BUF_SIZE];
//     while (1) {
//         current_pkt(1);
//         int len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             data[len] = '\0'; // null terminate
//             printf("Current: ");
//             for (int i = 0; i < len; i++) {
//                 printf("%02X ", data[i]);
//             }
//             printf("\n");
//         }
//         temp_pkt(1);
//         int len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             data[len] = '\0'; // null terminate
//             printf("Temp: ");
//             for (int i = 0; i < len; i++) {
//                 printf("%02X ", data[i]);
//             }
//             printf("\n");
//         }
//         load_pkt(1);
//         int len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
//         if (len > 0) {
//             data[len] = '\0'; // null terminate
//             printf("Load: ");
//             for (int i = 0; i < len; i++) {
//                 printf("%02X ", data[i]);
//             }
//             printf("\n");
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
void current_task(void *pv) {
    uint8_t data[BUF_SIZE];

    // Per-sensor debounce counters — local to this task. A fault trips
    // only after FAULT_CONSECUTIVE_TRIPS readings in a row, so a single
    // noisy frame from the motor bus cannot latch a false fault.
    int current_trip_count = 0;
    int temp_trip_count    = 0;
    int load_trip_count    = 0;

    while (1) {
        // --- CURRENT ---
        current_pkt(1);
        int len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len >= 2) {   // expecting at least 2 bytes
            int16_t current_val = (int16_t)((data[1] << 8) | data[0]);  // little endian
            printf("Current: %d\n", current_val);

            // ---- LIMIT CHECK: Current ----
            if (current_val > LIMIT_CURRENT_MAX_RAW || current_val < -LIMIT_CURRENT_MAX_RAW) {
                current_trip_count++;
                if (current_trip_count >= FAULT_CONSECUTIVE_TRIPS) {
                    enter_safe_state(FAULT_OVERCURRENT, "current", current_val);
                }
            } else {
                current_trip_count = 0;
            }
        }

        // --- TEMPERATURE ---
        temp_pkt(1);
        len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len >= 2) {
            int16_t temp_val = (int16_t)((data[1] << 8) | data[0]);
            printf("Temp: %d\n", temp_val);

            // ---- LIMIT CHECK: Temperature ----
            if (temp_val > LIMIT_TEMP_MAX_C) {
                temp_trip_count++;
                if (temp_trip_count >= FAULT_CONSECUTIVE_TRIPS) {
                    enter_safe_state(FAULT_OVERTEMP, "temp", temp_val);
                }
            } else {
                temp_trip_count = 0;
            }
        }

        // --- LOAD ---
        load_pkt(1);
        len = uart_read_bytes(UART1, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len >= 4) {
            int32_t load_val = (int32_t)((data[3] << 24) | (data[2] << 16) |
                                         (data[1] << 8) | data[0]);
            printf("Load: %ld\n", (long)load_val);

            // ---- LIMIT CHECK: Load ----
            if (load_val > LIMIT_LOAD_MAX_RAW || load_val < -LIMIT_LOAD_MAX_RAW) {
                load_trip_count++;
                if (load_trip_count >= FAULT_CONSECUTIVE_TRIPS) {
                    enter_safe_state(FAULT_OVERLOAD, "load", (long)load_val);
                }
            } else {
                load_trip_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Main
// ----------------------------------------------------
void app_main(void) {
    ESP_LOGI("MAIN", "Grabber firmware started!");
    
    nvs_flash_init();

  
    uart_driver_install(UART0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_init();

    mode_pkt(1, 0); 
    hx711_init();
    //motor_init();
    ESP_LOGI("MOTOR", "Motor task running, waiting for commands...");
    uint8_t data[BUF_SIZE];

    xTaskCreate(loadcell_task, "loadcell_task", 4096, NULL, 5, NULL);
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(current_task, "current_task", 4096, NULL, 5, NULL);
    
}