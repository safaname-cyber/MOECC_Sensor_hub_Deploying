/*   RESET FAULT
POS QUERY
GRABBER OPEN

GRABBER CLOSE
*/


#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// ==========================================
// FIRMWARE VERSION (SOP §16)
// ==========================================
#define FW_VERSION  "1.3.0"

// ==========================================
// CONFIGURATION & THRESHOLDS
// ==========================================
#define PKT_HEADER0 0xFF
#define PKT_HEADER1 0xFF
#define PKT_LENGTH  0x0A
#define INST_WRITE  0x03
#define INST_READ   0x02

// Motor Register Addresses (verified against official Feetech/Waveshare SMS_STS spec)
#define STS_MIN_ANGLE_L   0x09     // register 9   — minimum allowed position (low byte)
#define STS_MIN_ANGLE_H   0x0A     // register 10  — minimum allowed position (high byte)
#define STS_MAX_ANGLE_L   0x0B     // register 11  — maximum allowed position (low byte)
#define STS_MAX_ANGLE_H   0x0C     // register 12  — maximum allowed position (high byte)
#define STS_MODE          0x21     // register 33
#define STS_ACCEL         0x29     // register 41  — start of movement block
#define STS_PRESENT_POS_L 0x38     // register 56  — current position (low byte)
#define STS_PRESENT_POS_H 0x39     // register 57  — current position (high byte)
#define STS_VOLTAGE       0x3E     // register 62
#define STS_TEMP          0x3F     // register 63
#define STS_CURRENT       0x45     // register 69

// --- Safety Thresholds (SOP §7, §8.2 / Checklist B) ---
#define MAX_TEMP_C            75
#define MAX_CURRENT_MA        1000
#define MIN_VOLTAGE_V         0.0f     // TEMP: lowered for weak PSU testing. REVERT to 20.0f for production.
#define MAX_VOLTAGE_V         28.0f
#define MIN_VALID_VOLTAGE_V    1.0f
#define MAX_WEIGHT_G          5000.0f
#define MIN_WEIGHT_G         (-5000.0f)

#define FAULT_CONSECUTIVE_TRIPS    2
#define COMMS_TIMEOUT_TRIPS        5

// ---------------- HX711 Pins ----------------
#define LOADCELL_DOUT_PIN  GPIO_NUM_18
#define LOADCELL_SCK_PIN   GPIO_NUM_19

// ---------------- Motor Pins ----------------
#define MOTOR_TX_PIN   GPIO_NUM_7
#define MOTOR_RX_PIN   GPIO_NUM_8

// ---------------- UART ----------------
#define UART0       UART_NUM_0
#define UART1       UART_NUM_1
#define BUF_SIZE    1024

// ---------------- Motion Targets ----------------
#define POS_PHYSICAL_OPEN     0
#define POS_PHYSICAL_CLOSE    2048
#define MOTION_SPEED          2400
#define MOTION_ACCEL          50
#define MOTOR_ID              1

static const char *TAG = "GRABBER";

// ==========================================
// FAULT STATE (SOP §9.2)
// ==========================================
typedef enum {
    FAULT_NONE           = 0,
    FAULT_OVERCURRENT    = 1,
    FAULT_OVERTEMP       = 2,
    FAULT_OVERVOLTAGE    = 3,
    FAULT_UNDERVOLTAGE   = 4,
    FAULT_OVERWEIGHT     = 5,
    FAULT_SENSOR_NAN     = 6,
    FAULT_COMMS_LOST     = 7,
} fault_t;

static atomic_int g_fault = ATOMIC_VAR_INIT(FAULT_NONE);

static const char *fault_name(fault_t f) {
    switch (f) {
        case FAULT_NONE:         return "NONE";
        case FAULT_OVERCURRENT:  return "OVERCURRENT";
        case FAULT_OVERTEMP:     return "OVERTEMP";
        case FAULT_OVERVOLTAGE:  return "OVERVOLTAGE";
        case FAULT_UNDERVOLTAGE: return "UNDERVOLTAGE";
        case FAULT_OVERWEIGHT:   return "OVERWEIGHT";
        case FAULT_SENSOR_NAN:   return "SENSOR_NAN";
        case FAULT_COMMS_LOST:   return "COMMS_LOST";
        default:                 return "UNKNOWN";
    }
}

// HX711 globals
static float calibration_factor = -317.9f;
static long  offset = 0;
static float scale  = 1.0f;

// Forward decls
static void enter_safe_state(fault_t code, const char *src, long val);
static void Position_pkt(uint8_t motor_id, uint8_t acceleration,
                         uint16_t target_pos, uint16_t time, int16_t speed);
static bool read_motor_register(uint8_t id, uint8_t reg_addr, uint8_t length, uint8_t* out_data);

// ==========================================
// HARDWARE INIT
// ==========================================
void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART1, &uart_config);
    uart_set_pin(UART1, MOTOR_TX_PIN, MOTOR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    const uart_config_t uart0_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART0, &uart0_config);
}

// ==========================================
// HX711 LOAD CELL
// ==========================================
static void hx711_init() {
    gpio_config_t io_conf = { .mode = GPIO_MODE_INPUT, .pin_bit_mask = (1ULL << LOADCELL_DOUT_PIN) };
    gpio_config(&io_conf);
    gpio_config_t sck_conf = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << LOADCELL_SCK_PIN) };
    gpio_config(&sck_conf);
    gpio_set_level(LOADCELL_SCK_PIN, 0);
}

static int hx711_is_ready() { return (gpio_get_level(LOADCELL_DOUT_PIN) == 0); }

static long hx711_read() {
    while (!hx711_is_ready()) vTaskDelay(1);
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

static void hx711_tare(int times) { offset = hx711_read_average(times); }
static void hx711_set_scale(float scale_val) { scale = scale_val; }
static float hx711_get_units(int times) {
    long val = hx711_read_average(times) - offset;
    return (float)val / scale;
}

void loadcell_task(void *pv) {
    ESP_LOGI(TAG, "Initializing Load Cell...");
    hx711_set_scale(1.0f);
    hx711_tare(10);
    hx711_set_scale(calibration_factor);

    int weight_trip = 0;

    while (1) {
        float weight = hx711_get_units(5);

        if (isnan(weight) || isinf(weight)) {
            enter_safe_state(FAULT_SENSOR_NAN, "weight", 0);
        }
        else if (weight > MAX_WEIGHT_G || weight < MIN_WEIGHT_G) {
            if (++weight_trip >= FAULT_CONSECUTIVE_TRIPS) {
                enter_safe_state(FAULT_OVERWEIGHT, "weight", (long)weight);
            }
        } else {
            weight_trip = 0;
        }

        printf("[SCALE] Weight: %.2f g\n", weight);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==========================================
// MOTOR PROTOCOL
// ==========================================
static uint8_t checksum(uint8_t* data, int length) {
    uint16_t sum = 0;
    for (int i = 2; i < length - 1; i++) {
        sum += data[i];
    }
    return (~sum) & 0xFF;
}

void mode_pkt(uint8_t id, uint8_t mode){
    uint8_t pkt[8] = {
        PKT_HEADER0, PKT_HEADER1,
        id, 0x04, INST_WRITE,
        STS_MODE,
        mode,
        0
    };
    pkt[7] = checksum(pkt, sizeof(pkt));
    uart_write_bytes(UART1, (const char*)pkt, sizeof(pkt));
}

static void Position_pkt(uint8_t motor_id, uint8_t acceleration,
                         uint16_t target_pos, uint16_t time, int16_t speed) {
    uint16_t val = speed < 0 ? ((-speed) | (1 << 15)) : (uint16_t)speed;
    uint8_t pkt[14] = {
        PKT_HEADER0, PKT_HEADER1,
        motor_id, PKT_LENGTH, INST_WRITE,
        STS_ACCEL,
        acceleration,
        target_pos & 0xFF, (target_pos >> 8) & 0xFF,
        time       & 0xFF, (time       >> 8) & 0xFF,
        val        & 0xFF, (val        >> 8) & 0xFF,
        0
    };
    pkt[13] = checksum(pkt, sizeof(pkt));
    uart_write_bytes(UART1, (const char *)pkt, sizeof(pkt));
}

static void enter_safe_state(fault_t code, const char *src, long val) {
    int expected = FAULT_NONE;
    if (atomic_compare_exchange_strong(&g_fault, &expected, (int)code)) {
        ESP_LOGE(TAG, "!!! SAFE STATE: %s (src=%s val=%ld) !!!",
                 fault_name(code), src, val);
        Position_pkt(MOTOR_ID, 0, 0, 0, 0);
    }
}

// ==========================================
// MOTOR REGISTER READ
// ==========================================
bool read_motor_register(uint8_t id, uint8_t reg_addr, uint8_t length, uint8_t* out_data) {
    uint8_t req[8] = {PKT_HEADER0, PKT_HEADER1, id, 0x04, INST_READ, reg_addr, length, 0};
    req[7] = checksum(req, sizeof(req));

    uart_flush(UART1);
    uart_write_bytes(UART1, (const char*)req, sizeof(req));

    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t rx_buf[16];
    int len = uart_read_bytes(UART1, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(15));

    if (len < (int)(6 + length))           return false;
    if (rx_buf[0] != PKT_HEADER0)          return false;
    if (rx_buf[1] != PKT_HEADER1)          return false;
    if (rx_buf[2] != id)                   return false;

    uint16_t sum = 0;
    for (int i = 2; i < 5 + length; i++) sum += rx_buf[i];
    if (((~sum) & 0xFF) != rx_buf[5 + length]) return false;

    for (int i = 0; i < length; i++) out_data[i] = rx_buf[5 + i];
    return true;
}

// ==========================================
// DIAGNOSTIC HELPERS
// ==========================================

// Read current motor position. Returns -1 on failure.
static int32_t read_present_position(uint8_t id) {
    uint8_t buf[2];
    if (!read_motor_register(id, STS_PRESENT_POS_L, 2, buf)) return -1;
    return (int32_t)(buf[0] | (buf[1] << 8));
}

// One-shot: dump motor's actual configuration at boot.
// Tells us if MAX_ANGLE_LIMIT is clamping our position commands.
static void dump_motor_config(uint8_t id) {
    uint8_t buf[2];
    uint16_t min_angle = 0xFFFF, max_angle = 0xFFFF;
    uint8_t mode = 0xFF;
    int32_t pos = -1;
    bool got_min = false, got_max = false, got_mode = false, got_pos = false;

    ESP_LOGI("CONFIG", "------ Motor %d configuration ------", id);

    if (read_motor_register(id, STS_MIN_ANGLE_L, 2, buf)) {
        min_angle = (uint16_t)(buf[0] | (buf[1] << 8));
        got_min = true;
    }
    if (read_motor_register(id, STS_MAX_ANGLE_L, 2, buf)) {
        max_angle = (uint16_t)(buf[0] | (buf[1] << 8));
        got_max = true;
    }
    if (read_motor_register(id, STS_MODE, 1, buf)) {
        mode = buf[0];
        got_mode = true;
    }
    pos = read_present_position(id);
    got_pos = (pos >= 0);

    if (got_min) ESP_LOGI("CONFIG", "  MIN_ANGLE_LIMIT = %u", min_angle);
    else         ESP_LOGW("CONFIG", "  MIN_ANGLE_LIMIT = <read failed>");

    if (got_max) ESP_LOGI("CONFIG", "  MAX_ANGLE_LIMIT = %u", max_angle);
    else         ESP_LOGW("CONFIG", "  MAX_ANGLE_LIMIT = <read failed>");

    if (got_mode) ESP_LOGI("CONFIG", "  MODE = %d (0=joint/servo, 1=wheel)", mode);
    else          ESP_LOGW("CONFIG", "  MODE = <read failed>");

    if (got_pos) ESP_LOGI("CONFIG", "  PRESENT_POSITION = %ld", (long)pos);
    else         ESP_LOGW("CONFIG", "  PRESENT_POSITION = <read failed>");

    // Sanity check: warn if our target positions are outside the motor's allowed range.
    if (got_max && max_angle != 0) {
        if (POS_PHYSICAL_CLOSE > max_angle) {
            ESP_LOGW("CONFIG", "  *** POS_PHYSICAL_CLOSE (%d) exceeds MAX_ANGLE_LIMIT (%u) ***",
                     POS_PHYSICAL_CLOSE, max_angle);
            ESP_LOGW("CONFIG", "  *** Motor will clamp commands to %u ***", max_angle);
        }
        if (POS_PHYSICAL_OPEN < min_angle && got_min) {
            ESP_LOGW("CONFIG", "  *** POS_PHYSICAL_OPEN (%d) below MIN_ANGLE_LIMIT (%u) ***",
                     POS_PHYSICAL_OPEN, min_angle);
        }
    }

    ESP_LOGI("CONFIG", "------------------------------------");
}

// ==========================================
// FEEDBACK & SAFETY MONITORING
// ==========================================
void poll_all_feedback(uint8_t id) {
    uint8_t data[2];
    int16_t current_ma = 0;
    float   voltage    = 0.0f;
    int     temp_c     = 0;
    bool    got_c = false, got_v = false, got_t = false;

    if (read_motor_register(id, STS_CURRENT, 2, data)) {
        current_ma = (int16_t)((data[0] | (data[1] << 8)));
        got_c = true;
    }
    if (read_motor_register(id, STS_VOLTAGE, 1, data)) {
        voltage = data[0] / 10.0f;
        got_v = true;
    }
    if (read_motor_register(id, STS_TEMP, 1, data)) {
        temp_c = data[0];
        got_t = true;
    }

    static int comms_fail = 0;
    if (!got_c && !got_v && !got_t) {
        if (++comms_fail >= COMMS_TIMEOUT_TRIPS) {
            enter_safe_state(FAULT_COMMS_LOST, "motor_bus", comms_fail);
        }
        return;
    }
    comms_fail = 0;

    static int idle_counter = 0;
    if (current_ma > 50 || current_ma < -50) {
        printf("[MOTOR] >>> ACTIVE: %4d mA | Volt: %4.1f V | Temp: %2d C\n",
               current_ma, voltage, temp_c);
        idle_counter = 0;
    } else if (++idle_counter >= 5) {
        printf("[MOTOR]     Idle  : %4d mA | Volt: %4.1f V | Temp: %2d C\n",
               current_ma, voltage, temp_c);
        idle_counter = 0;
    }

    if (atomic_load(&g_fault) != FAULT_NONE) return;

    static int curr_trip = 0, temp_trip = 0, ov_trip = 0, uv_trip = 0;

    if (got_c && (current_ma > MAX_CURRENT_MA || current_ma < -MAX_CURRENT_MA)) {
        if (++curr_trip >= FAULT_CONSECUTIVE_TRIPS)
            enter_safe_state(FAULT_OVERCURRENT, "current", current_ma);
    } else curr_trip = 0;

    if (got_t && temp_c > MAX_TEMP_C) {
        if (++temp_trip >= FAULT_CONSECUTIVE_TRIPS)
            enter_safe_state(FAULT_OVERTEMP, "temp", temp_c);
    } else temp_trip = 0;

    if (got_v && voltage >= MIN_VALID_VOLTAGE_V) {
        if (voltage > MAX_VOLTAGE_V) {
            if (++ov_trip >= FAULT_CONSECUTIVE_TRIPS)
                enter_safe_state(FAULT_OVERVOLTAGE, "voltage", (long)(voltage * 10));
        } else ov_trip = 0;

        // Undervoltage check skipped while MIN_VOLTAGE_V = 0.0f (weak PSU workaround)
        if (MIN_VOLTAGE_V > 0.0f && voltage < MIN_VOLTAGE_V) {
            if (++uv_trip >= FAULT_CONSECUTIVE_TRIPS)
                enter_safe_state(FAULT_UNDERVOLTAGE, "voltage", (long)(voltage * 10));
        } else uv_trip = 0;
    }
}

void feedback_safety_task(void *pv) {
    while (1) {
        poll_all_feedback(MOTOR_ID);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ==========================================
// RESET FAULT
// ==========================================
static bool try_reset_fault(void) {
    fault_t active = (fault_t)atomic_load(&g_fault);
    if (active == FAULT_NONE) {
        ESP_LOGI("RESET", "No fault active, nothing to clear.");
        return true;
    }

    ESP_LOGI("RESET", "Attempting to clear fault=%s ...", fault_name(active));

    uint8_t data[2];
    int16_t cur_now = 0;
    float   v_now   = 0.0f;
    int     t_now   = 0;
    bool    got_c = false, got_v = false, got_t = false;

    if (read_motor_register(MOTOR_ID, STS_CURRENT, 2, data)) {
        cur_now = (int16_t)((data[0] | (data[1] << 8)));
        got_c = true;
    }
    if (read_motor_register(MOTOR_ID, STS_VOLTAGE, 1, data)) {
        v_now = data[0] / 10.0f;
        got_v = true;
    }
    if (read_motor_register(MOTOR_ID, STS_TEMP, 1, data)) {
        t_now = data[0];
        got_t = true;
    }

    if (!got_c && !got_v && !got_t) {
        ESP_LOGE("RESET", "Cannot reset: motor bus silent.");
        return false;
    }

    ESP_LOGI("RESET", "Live: I=%d mA, V=%.1f V, T=%d C", cur_now, v_now, t_now);

    switch (active) {
        case FAULT_OVERCURRENT:
            if (got_c && (cur_now > MAX_CURRENT_MA || cur_now < -MAX_CURRENT_MA)) {
                ESP_LOGE("RESET", "Still overcurrent (%d mA). Cannot clear.", cur_now);
                return false;
            }
            break;
        case FAULT_OVERTEMP:
            if (got_t && t_now > MAX_TEMP_C) {
                ESP_LOGE("RESET", "Still overtemp (%d C). Cannot clear.", t_now);
                return false;
            }
            break;
        case FAULT_OVERVOLTAGE:
            if (got_v && v_now > MAX_VOLTAGE_V) {
                ESP_LOGE("RESET", "Still overvoltage (%.1f V). Cannot clear.", v_now);
                return false;
            }
            break;
        case FAULT_UNDERVOLTAGE:
            if (got_v && v_now >= MIN_VALID_VOLTAGE_V && v_now < MIN_VOLTAGE_V) {
                ESP_LOGE("RESET", "Still undervoltage (%.1f V). Cannot clear.", v_now);
                return false;
            }
            break;
        case FAULT_COMMS_LOST:
            break;
        default:
            break;
    }

    atomic_store(&g_fault, FAULT_NONE);
    ESP_LOGI("RESET", "Fault cleared. System operational.");
    return true;
}

// ==========================================
// COMMAND TASK (UART0)
// ==========================================
void motor_task(void *pv) {
    ESP_LOGI("COMMAND", "Ready. Commands: GRABBER OPEN | GRABBER CLOSE | POS QUERY | RESET FAULT");
    uint8_t data[BUF_SIZE];
    bool fault_logged = false;
    uart_flush(UART0);

    while (1) {
        memset(data, 0, BUF_SIZE);
        int len = uart_read_bytes(UART0, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        data[len] = '\0';

        // -----------------------------------------------------------
        // RESET FAULT — handled BEFORE the fault gate.
        // -----------------------------------------------------------
        if (strstr((char*)data, "RESET FAULT") != NULL ||
            strstr((char*)data, "reset fault") != NULL) {
            ESP_LOGI("COMMAND", "RESET FAULT requested");
            if (try_reset_fault()) {
                fault_logged = false;
            }
            continue;
        }

        // -----------------------------------------------------------
        // POS QUERY — handled BEFORE the fault gate so it works even
        // when a fault is latched (useful for diagnosing where motor stopped).
        // -----------------------------------------------------------
        if (strstr((char*)data, "POS QUERY") != NULL ||
            strstr((char*)data, "pos query") != NULL) {
            int32_t pos = read_present_position(MOTOR_ID);
            if (pos >= 0) {
                ESP_LOGI("QUERY", "Current motor position: %ld", (long)pos);
            } else {
                ESP_LOGE("QUERY", "Failed to read motor position");
            }
            continue;
        }

        // -----------------------------------------------------------
        // Fault gate
        // -----------------------------------------------------------
        fault_t active = (fault_t)atomic_load(&g_fault);
        if (active != FAULT_NONE) {
            if (!fault_logged) {
                ESP_LOGE("COMMAND", "Locked: fault=%s. Send 'RESET FAULT' to clear.",
                         fault_name(active));
                fault_logged = true;
            }
            continue;
        }
        fault_logged = false;

        // -----------------------------------------------------------
        // Motion commands
        // -----------------------------------------------------------
        if (strstr((char*)data, "GRABBER OPEN") != NULL || strstr((char*)data, "grabber open") != NULL) {
            ESP_LOGI("MOTOR", "Executing GRABBER OPEN -> pos %d", POS_PHYSICAL_OPEN);
            Position_pkt(MOTOR_ID, MOTION_ACCEL, POS_PHYSICAL_OPEN, 0, MOTION_SPEED);

        } else if (strstr((char*)data, "GRABBER CLOSE") != NULL || strstr((char*)data, "grabber close") != NULL) {
            ESP_LOGI("MOTOR", "Executing GRABBER CLOSE -> pos %d", POS_PHYSICAL_CLOSE);
            Position_pkt(MOTOR_ID, MOTION_ACCEL, POS_PHYSICAL_CLOSE, 0, MOTION_SPEED);

        } else {
            ESP_LOGW("COMMAND", "Unknown command (ignored): %s", data);
        }
    }
}

// ==========================================
// APP MAIN
// ==========================================
void app_main(void) {
    ESP_LOGI("MAIN", "Grabber firmware v%s (built %s %s)",
             FW_VERSION, __DATE__, __TIME__);

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGW("MAIN", "nvs_flash_init returned %d, continuing", err);
    }
    uart_init();
    hx711_init();

    // Set joint/servo mode before reading config (in case mode register reflects intent)
    mode_pkt(MOTOR_ID, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // ============================================================
    // DIAGNOSTIC: dump motor configuration at boot
    // ============================================================
    dump_motor_config(MOTOR_ID);

    BaseType_t r;
    r = xTaskCreate(loadcell_task,        "loadcell_task",  4096, NULL, 4, NULL);
    if (r != pdPASS) { ESP_LOGE("MAIN", "loadcell_task create failed"); while(1) vTaskDelay(1000); }

    r = xTaskCreate(feedback_safety_task, "feedback_task",  4096, NULL, 5, NULL);
    if (r != pdPASS) { ESP_LOGE("MAIN", "feedback_task create failed"); while(1) vTaskDelay(1000); }

    r = xTaskCreate(motor_task,           "motor_task",     4096, NULL, 6, NULL);
    if (r != pdPASS) { ESP_LOGE("MAIN", "motor_task create failed"); while(1) vTaskDelay(1000); }

    ESP_LOGI("MAIN", "Grabber ready.");
}