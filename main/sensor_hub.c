/* =============================================================================
 * sensor_hub.c  —  Sensor Hub Deployment Controller (MoECC QRDI)
 * -----------------------------------------------------------------------------
 * Target   : ESP32-S3-WROOM-1, ESP-IDF v6.0.x, FreeRTOS
 * Version  : 2.0.0  (MAJOR.MINOR.PATCH)                       [SOP §16 / F]
 * Hardware : 2 x SmartElex 15S (PWM 20 kHz + DIR), 2 x DENSO 12 V motors,
 *            2 x Orange 3806-OPTI-1000-AB-OC encoders (1000 PPR, NPN open
 *            collector, 5-24 V supply; PCNT x4 = 4000 cpr),
 *            4 x limit switches (active-low): per arm LS1 = stow, LS2 = deploy,
 *            2 x lock servos (LEDC 50 Hz, one per arm; set only at deploy).
 *            No hardware E-stop: 'X' (stop) is the software E-stop path.
 *
 * -----------------------------------------------------------------------------
 * TASKS (all statically allocated; each subscribes itself to the TWDT)
 *   name          prio  stack  period / blocking
 *   safety_task    12   3072   100 ms. Watches alive counters:
 *                              motion_task silent 500 ms -> both PWMs forced
 *                              low + trip; status_task silent 1 s -> trip.
 *                              A trip stops all motion; only reset (R) clears
 *                              it (refused if the PWM pins had to be detached:
 *                              reboot).
 *   motion_task    10   6144   10 ms (+ limit ISR fast stop). The ONLY code
 *                              driving motors, DIR, servos and arm states:
 *                              commands, limit completion, move window /
 *                              TIMEOUT, approach slow-down, ramp, locks, NVS.
 *   status_task     8   4096   20 ms, on the snapshot: STALL, WRONG_DIR,
 *                              OVERTRAVEL, SWITCH_STUCK, WRONG_LIMIT,
 *                              BOTH_LIMITS -> fault request to motion;
 *                              progress (500 ms), status (5 s), DRIFT,
 *                              POSITION_LOST, POSITION_UNKNOWN, health.
 *   cmd_task        7   4096   blocks on line queue: parse, validate,
 *                              status replies from the snapshot, queue
 *                              motion commands.
 *   uart_task       6   4096   10 ms poll: RX of UART0 + UART1 into lines
 *                              (UART0 echo, UART1 idle line end) and the ONLY
 *                              writer of output lines.
 *   Data flow: uart -> line_q -> cmd -> cmd_q -> motion -> evt_q -> uart
 *              status -> req_q -> motion (fault / position lost / save)
 *              any task -> evt_q / log_q -> uart
 *   Shared data [SOP §9.2 / D]: motion publishes a snapshot every tick
 *   (s_snap_mtx) read by status and cmd; motor outputs under s_motor_mtx
 *   (motion, and safety when forcing low); s_trip / alive counters atomic.
 *   A status-side fault request carries the move sequence number, so it
 *   is ignored if that move already ended.
 *
 * INTERRUPTS
 *   Limit-switch GPIO any-edge ISR: only vTaskNotifyGiveFromISR(motion_task),
 *   which immediately sets PWM 0 if a driving arm's target switch is closed.
 *
 * MEMORY
 *   No malloc/free in application code. Queues, tasks and mutexes are static.
 *   Documented exception [SOP §8.1]: ESP-IDF drivers (UART, MCPWM, PCNT,
 *   NVS, LEDC, GPIO ISR service) allocate internally ONCE at init.
 *   cJSON is NOT used (it mallocs); a fixed-buffer flat JSON parser is used.
 *
 * TIMING                                                [SOP §4.2, §7 / B, E]
 *   Motion loop 100 Hz: switches debounced over 50 ms (5 samples), encoder
 *   edges counted in hardware (PCNT). 5000 rpm x 4000 cpr = 333 kcount/s ->
 *   3333 counts/tick, inside the PCNT +/-30000 window (static-asserted).
 *   TWDT timeout 3 s.
 *
 * COMMS-LOSS POLICY                                          [SOP §6 / C]
 *   Every move self-terminates (limit / time / timeout / fault), so silence
 *   from the host is NOT a fault. The controller sends STATUS every 5 s.
 *
 * -----------------------------------------------------------------------------
 * FLAGS (per arm, saved in NVS)
 *   f1 = 1 : a DEPLOY was stopped part-way (arm is between the switches)
 *   f2 = 1 : a STOW   was stopped part-way
 *   Set by stop / start-time-elapsed / fault while driving. Both cleared when
 *   a limit switch is reached.
 *
 * MOVE RULES (each arm independent)
 *   Deploy (D, D1, D2), checked in this order:
 *     1. fault latched or safety trip   -> rejected (send R)
 *     2. moving                         -> IN_PROGRESS (same dir) / BUSY
 *     3. speed < MIN_SPEED (0.15)       -> rejected SPEED_TOO_LOW
 *     4. f1 = 1                         -> start forward (continuing)
 *     5. servo_err                      -> warn LOCK_NOT_OPEN, no move
 *     6. LS1 closed and LS2 open        -> start forward (fresh)
 *     7. both open                      -> start forward if f1 or f2 or
 *                                          pos_valid, else warn
 *                                          POSITION_UNKNOWN, no move
 *     8. LS2 closed                     -> ALREADY_DEPLOYED (lock set)
 *        both closed                    -> SWITCH_FAULT (fault BOTH_LIMITS)
 *     Before any start: pos_valid, stroke_valid, LS2 open and
 *     count > stroke + max(10 %, 0.5 turn) -> rejected POSITION_CONFLICT.
 *   Stow (S, S1, S2): mirror — f2 continues, LS2 closed / LS1 open starts
 *     fresh, LS1 closed -> ALREADY_STOWED. Before any start: pos_valid,
 *     LS1 open and count < -0.5 turn -> rejected POSITION_CONFLICT.
 *   Every start first opens the arm's lock and waits until it reads
 *   UNLOCKED before the motor may run.
 *   Start (G, G1, G2 / "start"): resumes a stopped move.
 *     f1 = f2 = 0 -> rejected NOTHING_TO_RESUME. No direction -> forward if
 *     f1, reverse if f2. Target switch closed -> ALREADY_DEPLOYED /
 *     ALREADY_STOWED. POSITION_CONFLICT checked as for deploy / stow.
 *     time 1-180 s, speed 0.15-1.0 (this move only), else
 *     TIME_OUT_OF_RANGE / SPEED_OUT_OF_RANGE. With a time it stops after
 *     that time like X (TIME_ELAPSED). Keeps all fault checks.
 *   Fault while driving: f1 / f2 set by direction, time_used updated,
 *     pos_valid kept, NVS saved -> resumable after R.
 *   Reset (R): keeps f1 / f2, re-reads the switches; refused while both
 *     switches are closed.
 *   Idle: count moving > 0.05 turn with the motor off -> DRIFT warning
 *     (once per stop). POSITION_LOST (switch released while idle) keeps
 *     pos_valid: the encoder still tracks the arm.
 *   Stop (X): halts, saves f1/f2, run time and encoder position to NVS.
 *   Deploy reached -> lock set. Stow reached -> lock open, position = 0.
 *   No pause command.
 *
 * BOOT (per arm, boot_classify)
 *   LS1 closed  -> STOWED, count 0, pos_valid 1, f1/f2/time_used cleared.
 *   LS2 closed  -> DEPLOYED, f1/f2/time_used cleared, count = stroke if
 *                  stroke_valid, pos_valid = stroke_valid || record valid.
 *   both open   -> valid record: count/f1/f2 restored, pos_valid 1,
 *                  STOPPED if f1 or f2 else UNKNOWN; no record: pos_valid 0,
 *                  UNKNOWN.
 *   both closed -> FAULT BOTH_LIMITS.
 *   Boot never moves a motor; locks are set to match (locked only at LS2).
 *
 * MOVE WINDOW (per move leg)
 *   continuing (deploy & f1, stow & f2): window = timeout - time_used
 *   fresh move or reversal            : window = timeout (default 180 s,
 *                                       NVS, T<s> / set_timeout)
 *   start command with a time         : window = that time
 *   window < 2 s -> rejected WINDOW_EXHAUSTED. SHUB_BENCH_TEST: window is
 *   at least 120 s (except a start command with a time).
 *   End of window: TIMEOUT fault (start with a time: stop, TIME_ELAPSED).
 *   ETA = remaining counts / (rate_cps x speed) when pos_valid, stroke_valid
 *   and rate_cps are known; in the ack, WINDOW_SHORT warning if ETA > window.
 *
 * WHILE DRIVING
 *   Approach: remaining < max(10 % stroke, 1 turn) -> duty capped at 25 %
 *     (forward needs stroke_valid, reverse needs pos_valid).
 *   Overtravel (with pos_valid): forward count > stroke + max(10 %, 0.5 turn)
 *     (MAX_STROKE_COUNTS until learned), reverse count < -0.5 turn ->
 *     OVERTRAVEL. Without a position reference: travel > MAX_STROKE_COUNTS.
 *   Every 5 s the NVS record is saved as if stopped now (flag for this
 *   direction, time_used incl. this leg, count).
 *   Progress (500 ms): pos, % of stroke, duty, elapsed, window left, ETA.
 *   Stall / wrong-direction / switch checks as before.
 *
 * COMPLETION
 *   LS2: ENCODER_MISMATCH warning if |count - stroke| > 0.1 turn; with
 *     pos_valid stroke = count (learned), else count = stroke if known.
 *   LS1: ENCODER_MISMATCH warning if |count| > 0.1 turn; count = 0,
 *     pos_valid = 1.
 *   Both: rate_cps learned (legs > 1 s: counts / integral of duty dt),
 *   f1 / f2 / time_used cleared, NVS saved, lock set at LS2 / open at LS1.
 *
 * STOP (X, X1, X2; default both)
 *   PWM 0 at once. If the drive had started: f1 (deploy) or f2 (stow) = 1,
 *   the leg is added to time_used, stop_time = leg, STOPPED, NVS saved.
 *   Before the motor started (lock / reversal wait): cancelled, f1/f2 kept,
 *   state re-read from the switches. Reply carries pos, pct, f1, f2.

 * -----------------------------------------------------------------------------
 * PROTOCOL / ICD (both UARTs, 115200 8N1)
 *   Line end: UART1 newline / CR / 200 ms idle. UART0 (human console)
 *             newline / CR only, with local echo + backspace.
 *   Reply style: a letter command gets TEXT replies, a JSON command gets
 *   JSON replies (on UART0 + UART1). Async events (limit reached, progress,
 *   faults, status heartbeat): UART0 text, UART1 JSON, or the format of the
 *   command that started the move. Logs: "# ..." lines on UART0 only.
 *
 *   COMMANDS               letter (1 = port, 2 = stbd, none = both)
 *     deploy               D  D1  D2
 *     stow                 S  S1  S2
 *     start (resume)       G  G1  G2   [+ D|S][,<sec 1-180>[,<speed % 15-100>]]
 *     stop                 X  X1  X2
 *     reset fault / trip   R  R1  R2
 *     lock / unlock        L  L1  L2 / U  U1  U2
 *     default speed        V<0..100>
 *     move timeout         T<10..900>   (seconds, NVS)
 *     status               ?
 *   JSON ("side":"port|stbd|starboard|both", default both):
 *     {"cmd":"deploy|stow|stop|reset|lock|unlock","side":"port"}
 *     {"cmd":"start","side":"port","direction":"forward|reverse",
 *      "time":1-180,"speed":0.15-1.0}   direction / time / speed optional
 *     {"cmd":"set_speed","value":0.6}   {"cmd":"set_timeout","value":180}
 *     {"cmd":"status"}
 *     legacy: {"command":"Deploy|Retract"}  {"set_speed":0.6}
 *             {"command":"Pause"} -> stop (pause does not exist)
 *
 *   PER-ARM REPLY / EVENT (one per arm addressed)
 *     text: <msg> side=<s> state=<S> lock=<L> f1= f2= ls1= ls2= pos=<m> m
 *           pct=<p> t=<s> s [fault=<F>] [SERVO_ERR]
 *     JSON: {"evt":E,"cmd":C,"side":"port|stbd","ok":true|false,"code":K,
 *            "state":S,"lock":L,"f1":0|1,"f2":0|1,"ls1":0|1,"ls2":0|1,
 *            "pos_m":m,"pct":0..100|-1,"time_s":t,"servo_err":0|1,
 *            "fault":F,"msg":M}
 *     code K = leading CODE of msg, "OK" for "Ack: ..." replies.
 *   evt         code(s)                     meaning
 *     ack       OK                          accepted; msg has speed, window,
 *                                           ETA ("ETA unknown" if not known)
 *     rejected  FAULT_LATCHED               fault latched - send R
 *               SAFETY_TRIP                 safety task cut the drive - R
 *               SPEED_TOO_LOW               speed < 0.15
 *               POSITION_CONFLICT           encoder past the target end with
 *                                           its switch open
 *               WINDOW_EXHAUSTED            < 2 s of the move window left
 *               NOTHING_TO_RESUME           start with f1 = f2 = 0
 *     warn      POSITION_UNKNOWN            both switches open, no f1/f2,
 *                                           pos_valid = 0: no move
 *               LOCK_NOT_OPEN               servo_err: lock state unknown,
 *                                           no move (also cancels a start)
 *               SAFETY_TRIP                 trip during the lock / rest wait:
 *                                           start cancelled, send R
 *               WINDOW_SHORT                ETA > window (move still runs)
 *               ENCODER_MISMATCH            count off by > 0.1 turn at LS1
 *                                           (0) or LS2 (stroke)
 *               DRIFT                       count moved > 0.05 turn with the
 *                                           motor off
 *               STROKE_OUT_OF_RANGE         stroke not learned (bad count)
 *               BENCH_TEST / ABNORMAL_RESET boot warnings
 *     in_progress IN_PROGRESS               already moving that way
 *     busy      BUSY                        moving the other way - stop first
 *     deployed  DEPLOYED / ALREADY_DEPLOYED reached / already at LS2
 *     stowed    STOWED / ALREADY_STOWED     reached / already at LS1
 *     stopped   OK ("Ack: Stopped") / TIME_ELAPSED
 *     fault     FAULT / SWITCH_FAULT / SERVO_ERR   (fault= gives the reason:
 *               BOTH_LIMITS WRONG_LIMIT SWITCH_STUCK STALL WRONG_DIR
 *               OVERTRAVEL TIMEOUT SAFETY)
 *     locked / unlocked   LOCKED / UNLOCKED
 *     lock_denied LOCK_DENIED  unlock_denied UNLOCK_DENIED
 *     reset_denied RESET_DENIED (both switches closed)
 *     position_lost POSITION_LOST (switch released while idle)
 *     progress  PROGRESS: + "duty","elapsed_s","window_left_s","eta_s"
 *     boot      BOOT (one per arm after the boot line)
 *   STATUS (? / {"cmd":"status"}, and every 5 s): one per arm +
 *     {"evt":"status","side":"port|stbd","code":"STATUS","state","lock",
 *      "f1","f2","pos_valid","count","pct","stroke","stroke_valid",
 *      "time_used_s","ls1","ls2","pos_m","rate_cps","servo_err","fault"}
 *     {"evt":"status","side":"sys","code":"STATUS","fw","speed","timeout_s",
 *      "uptime_s","txdrop","bench"}
 *   SYSTEM replies:
 *     {"evt":"ack","cmd":"set_speed","code":"OK","speed",...}
 *     {"evt":"ack|rejected","cmd":"set_timeout","code":"OK|TIMEOUT_OUT_OF_RANGE",...}
 *     {"evt":"rejected","cmd":"start","code":"TIME_OUT_OF_RANGE|SPEED_OUT_OF_RANGE"}
 *     {"evt":"error","code":"UNKNOWN_COMMAND"}  {"evt":"busy","code":"BUSY_QUEUE"}
 *     {"evt":"failsafe","code":"FAILSAFE"}  {"evt":"boot",...}
 *   Text replies carry the same codes as the first word of the line.
 *
 * NVS (namespace "storage")
 *   "saved_speed" float blob   "timeout_s" u32
 *   "<arm>_rec"   blob arm_rec_t, one per arm (<arm> = port | stbd):
 *                 version, f1, f2, time_used_ms, count, stroke, stroke_valid,
 *                 rate_cps, stop_time_ms, pos_valid + CRC32. Written only
 *                 when it changed: on stop / limit / fault / DRIFT /
 *                 POSITION_LOST and every 5 s while driving. At load the whole record is
 *                 discarded (defaults used) if size, version, CRC or any value
 *                 is out of range. Old v1 keys (_cnt _dcnt _flags _tms) are
 *                 erased at boot.
 *   rate_cps     : learned encoder rate (counts/s) at 100 % duty, from
 *                  steady-speed driving (not learned in SHUB_BENCH_TEST).
 *   stop_time_ms : length of the driven segment that the last stop / fault
 *                  interrupted.
 *
 * BENCH CALIBRATION REQUIRED (search "CALIBRATE")
 *   DRUM_DIA_M, GEAR_RATIO, MAX_STROKE_COUNTS, STALL_MIN_COUNTS,
 *   SWITCH_RELEASE_COUNTS, per-arm
 *   deploy_dir_level and enc_sign, servo pulse widths and settle time.
 * ========================================================================== */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc.h"
#include "soc/usb_serial_jtag_reg.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"

/* =============================================================================
 * Version / identity                                         [SOP §16 / F]
 * ========================================================================== */
#define FW_VERSION          "2.0.0"
#define FW_BUILD_DATE       __DATE__ " " __TIME__
static const char *TAG = "SHUB";

/* =============================================================================
 * Pin map (ESP32-S3-WROOM-1)
 *   signal     port     stbd
 *   ENC A      GPIO8    GPIO20
 *   ENC B      GPIO19   GPIO3
 *   PWM        GPIO40   GPIO37
 *   DIR        GPIO41   GPIO38
 *   SERVO      GPIO35   GPIO47
 *   LS1 stow   GPIO4    GPIO6
 *   LS2 deploy GPIO5    GPIO7
 *   UART0 console 43/44, UART1 system TX 17 / RX 16.
 * Notes:
 *   GPIO19/20 are the native USB D-/D+ pads: hal_usb_pads_release() detaches
 *     the USB-Serial-JTAG PHY at boot so they work as encoder inputs. The
 *     native USB port (and its console copy) is then unavailable; flash and
 *     monitor through the UART port.
 *   GPIO35/37 (and 36) are used by octal PSRAM / octal flash on WROOM-1
 *     N*R8 / N*R16 modules — this map needs a module WITHOUT octal PSRAM.
 *   GPIO3 is a strapping pin (JTAG source select; ignored unless the eFuse
 *     is burned). GPIO38 drives the RGB LED on DevKitC-1 v1.1 boards.
 * ========================================================================== */
typedef enum { ARM_PORT = 0, ARM_STBD = 1, ARM_COUNT = 2 } arm_id_t;

typedef struct {
    const char *name;           /* "port" / "stbd"                           */
    gpio_num_t  pwm_gpio;       /* SmartElex 15S PWM input (10k pull-down!)   */
    gpio_num_t  dir_gpio;       /* SmartElex 15S DIR input                    */
    gpio_num_t  lim_deploy_gpio;/* LS2 deploy limit, active-low               */
    gpio_num_t  lim_stow_gpio;  /* LS1 stow limit, active-low                 */
    gpio_num_t  enc_a_gpio;     /* encoder A (4.7k pull-up to 3V3)            */
    gpio_num_t  enc_b_gpio;     /* encoder B (4.7k pull-up to 3V3)            */
    uint32_t    deploy_dir_level; /* DIR level that DEPLOYS this arm  CALIBRATE */
    int32_t     enc_sign;         /* +1/-1 so deploying increases count CALIBRATE */
    gpio_num_t  servo_gpio;     /* lock servo signal (LEDC)                   */
    ledc_channel_t servo_ch;    /* LEDC channel for this arm's lock servo     */
} arm_hw_t;

static const arm_hw_t ARM_HW[ARM_COUNT] = {
    /* name   PWM          DIR          LS2 DEP      LS1 STOW     ENC_A        ENC_B        DEP  SIGN  SERVO        SERVO_CH */
    { "port", GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_5,  GPIO_NUM_4,  GPIO_NUM_8,  GPIO_NUM_19, 1U,  1,    GPIO_NUM_35, LEDC_CHANNEL_0 },
    /* Arms are mirrored: starboard deploy level / encoder sign must be
     * confirmed on the bench before first powered run.          CALIBRATE */
    { "stbd", GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_7,  GPIO_NUM_6,  GPIO_NUM_20, GPIO_NUM_3,  1U,  1,    GPIO_NUM_47, LEDC_CHANNEL_1 },
};

#define UART_FLASH_PORT     UART_NUM_0          /* default pins GPIO43/44     */
#define UART_SYS_PORT       UART_NUM_1
#define UART_SYS_TX_GPIO    GPIO_NUM_17
#define UART_SYS_RX_GPIO    GPIO_NUM_16
#define UART_BAUD           115200
#define UART_RX_BUF_BYTES   1024
#define UART_TX_BUF_BYTES   1024

/* =============================================================================
 * Tunables
 * ========================================================================== */
#define CONTROL_PERIOD_MS       10U
#define DEBOUNCE_SAMPLES        5U          /* 5 x 10 ms = 50 ms (old firmware) */
#define RAMP_TIME_MS            300U        /* 0 -> 100 % duty                  */
#define DIR_DEADTIME_MS         200U
#define STALL_GRACE_MS          400U
#define STALL_WINDOW_MS         200U
/* Count thresholds are written per 400 cpr and scaled to ENC_CPR so the
 * physical travel they represent does not change with encoder resolution. */
#define STALL_MIN_COUNTS        (8 * ENC_CPR / 400)     /* CALIBRATE @ MIN_SPEED: expected/4 */
#define SWITCH_RELEASE_COUNTS   (400 * ENC_CPR / 400)   /* CALIBRATE: travel to clear start switch */
#define WRONG_DIR_COUNTS        (40 * ENC_CPR / 400)    /* counts the wrong way -> fault    */
#define MAX_STROKE_COUNTS       (6000 * ENC_CPR / 400)  /* CALIBRATE: full stroke + margin  */

/* Move timeout (total run time of one move, across stop/continue) */
#define MOVE_TIMEOUT_DEFAULT_S  180U
#define MOVE_TIMEOUT_MIN_S      10U
#define MOVE_TIMEOUT_MAX_S      900U

/* Encoder position logic (stow = 0, deploy position learned at deploy limit) */
#define HALF_TURN_COUNTS        (ENC_CPR / 2)
#define APPROACH_DUTY_PCT       25.0f       /* duty cap near the end of travel   */
#define MISMATCH_COUNTS         (ENC_CPR / 10)  /* 0.1 turn: ENCODER_MISMATCH    */
#define DRIFT_COUNTS            (ENC_CPR / 20)  /* 0.05 turn idle: DRIFT warning */

/* Start command limits */
#define START_TIME_MIN_S        1.0f
#define START_TIME_MAX_S        180.0f
#define START_SPEED_MIN         0.15f
#define START_SPEED_MAX         1.0f

/* Move window (time allowed for one move leg) */
#define WINDOW_MIN_MS           2000U       /* below -> WINDOW_EXHAUSTED         */
#define BENCH_WINDOW_MIN_MS     120000U     /* SHUB_BENCH_TEST: window minimum   */

/* Stow start refused if the arm reads this far behind stow (0.5 turn) */
#define STOW_CONFLICT_COUNTS    HALF_TURN_COUNTS

/* Rate learning (rate_cps = counts/s at 100 % duty), at move completion */
#define RATE_MIN_MOVE_MS        1000U       /* learn only from moves longer than */
#define RATE_EMA_DIV            4           /* new = old + (meas - old) / 4      */
#define RATE_MAX_CPS            ((uint32_t)((ENC_MAX_RPM / 60) * ENC_CPR))

#define ARM_REC_VERSION         2U          /* v2: pos_valid stored (v1 discarded once) */
#define TIME_MAX_MS             (MOVE_TIMEOUT_MAX_S * 1000U)   /* record limit for time fields */

/* BENCH TEST ONLY — encoder turned by hand, no motor load.
 * 1 = STALL / WRONG_DIR / OVERTRAVEL / SWITCH_STUCK checks OFF (timeout
 *     still applies). Limit-switch checks (BOTH_LIMITS, WRONG_LIMIT) stay ON.
 * NEVER fly with 1: a jammed or runaway arm will not be detected.
 * Set here or build with: idf.py -DSHUB_BENCH_TEST=1 build              */
#ifndef SHUB_BENCH_TEST
#define SHUB_BENCH_TEST         0
#endif
#define MIN_SPEED               0.15f
#define DEFAULT_SPEED           0.50f
#define SPEED_SAVE_EPS          0.005f
#define PROGRESS_PERIOD_MS      500U
#define HEARTBEAT_PERIOD_MS     5000U
#define NVS_SAVE_PERIOD_MS      5000U
#define TWDT_TIMEOUT_MS         3000U

/* Safety supervisor (alive counters) */
#define SAFETY_PERIOD_MS        100U
#define MOTION_SILENT_MS        500U        /* motion alive counter stuck -> trip, PWM low */
#define STATUS_SILENT_MS        1000U       /* status alive counter stuck -> trip */

/* Status monitor (fault checks on the snapshot, events) */
#define STATUS_PERIOD_MS        20U
#define STACK_REPORT_PERIOD_MS  60000U
#define STACK_LOW_BYTES         512U

#define SERVO_FREQ_HZ           50U
#define SERVO_PERIOD_US         20000U
#define SERVO_LEDC_BITS         14U
#define SERVO_LOCK_US           1000U       /* CALIBRATE                        */
#define SERVO_UNLOCK_US         2000U       /* CALIBRATE                        */
#define LOCK_SETTLE_MS          800U        /* CALIBRATE                        */

#define MCPWM_RES_HZ            10000000UL  /* 10 MHz tick                      */
#define PWM_FREQ_HZ             20000UL     /* old firmware value               */
#define MCPWM_PERIOD_TICKS      (MCPWM_RES_HZ / PWM_FREQ_HZ)   /* 500           */

#define PCNT_HIGH_LIMIT         30000
#define PCNT_LOW_LIMIT          (-30000)
#define PCNT_GLITCH_NS          1000U

/* Encoder calibration — same method as the winch external encoder. */
#define ENC_PPR                 1000                     /* 3806-OPTI-1000-AB-OC */
#define ENC_CPR                 (ENC_PPR * 4)            /* 4000 counts/rev     */
#define DRUM_DIA_M              0.075f                   /* CALIBRATE (winch value) */
#define GEAR_RATIO              1.0f                     /* CALIBRATE: encoder rev per drum rev */
#define METERS_PER_COUNT        ((3.14159265f * DRUM_DIA_M) / ((float)ENC_CPR * GEAR_RATIO))
#define ENC_MAX_RPM             5000                     /* encoder rating (3806 series) */

#define LINE_QUEUE_LEN          4U
#define CMD_QUEUE_LEN           8U
#define REQ_QUEUE_LEN           8U
#define EVT_QUEUE_LEN           24U
#define EVT_RESERVED_SLOTS      6U          /* kept free for critical events    */
#define LOG_QUEUE_LEN           16U
#define RX_LINE_MAX             512U        /* old firmware LINE_BUF_SIZE       */
#define RX_IDLE_END_MS          200U        /* UART1 idle line end              */
#define UART_POLL_MS            10U
#define TX_TEXT_MAX             256U
#define TX_JSON_MAX             384U
#define LOG_LINE_MAX            160U

#define SAFETY_STACK_BYTES      3072U
#define MOTION_STACK_BYTES      6144U
#define CMD_STACK_BYTES         4096U
#define UART_STACK_BYTES        4096U
#define STATUS_STACK_BYTES      4096U
#define SAFETY_PRIO             12U
#define MOTION_PRIO             10U
#define STATUS_PRIO             8U          /* runs the fault checks          */
#define CMD_PRIO                7U
#define UART_PRIO               6U

/* ---- compile-time invariants / overflow analysis          [SOP §7, §8.1] */
_Static_assert(ENC_CPR == ENC_PPR * 4, "x4 quadrature");
_Static_assert(DEBOUNCE_SAMPLES * CONTROL_PERIOD_MS == 50U, "debounce must stay 50 ms (old firmware)");
_Static_assert(STALL_WINDOW_MS % CONTROL_PERIOD_MS == 0U, "stall window multiple of tick");
_Static_assert(STALL_GRACE_MS > RAMP_TIME_MS, "stall grace must cover the soft-start ramp");
_Static_assert(RAMP_TIME_MS >= CONTROL_PERIOD_MS, "ramp step must be <= 100 %");
_Static_assert(MCPWM_PERIOD_TICKS >= 100UL && MCPWM_PERIOD_TICKS <= 65535UL, "MCPWM period range");
_Static_assert((ENC_MAX_RPM / 60) * ENC_CPR * (int)CONTROL_PERIOD_MS / 1000 < PCNT_HIGH_LIMIT,
               "max counts per tick must fit the PCNT window");
_Static_assert(MAX_STROKE_COUNTS > 0 && MAX_STROKE_COUNTS < (INT32_MAX / 4), "stroke counts range");
_Static_assert(SWITCH_RELEASE_COUNTS < MAX_STROKE_COUNTS, "release travel inside stroke");
_Static_assert(EVT_RESERVED_SLOTS < EVT_QUEUE_LEN, "reserved event slots");
_Static_assert(SERVO_UNLOCK_US < SERVO_PERIOD_US && SERVO_LOCK_US < SERVO_PERIOD_US, "servo pulse < period");
_Static_assert(MOTION_SILENT_MS >= 5U * CONTROL_PERIOD_MS, "motion silent window must span several ticks");
_Static_assert(STATUS_SILENT_MS >= 5U * STATUS_PERIOD_MS, "status silent window must span several loops");
_Static_assert(STALL_WINDOW_MS % STATUS_PERIOD_MS == 0U, "stall window multiple of status period");
_Static_assert(PROGRESS_PERIOD_MS % STATUS_PERIOD_MS == 0U, "progress multiple of status period");

/* =============================================================================
 * Types
 * ========================================================================== */
typedef enum {
    ST_UNKNOWN = 0,   /* between switches, no f1/f2: position unknown          */
    ST_STOWED,
    ST_DEPLOYED,
    ST_STARTING,      /* accepted: waiting for unlock settle / dir dead-time  */
    ST_DEPLOYING,
    ST_STOWING,
    ST_STOPPED,       /* stopped part-way, f1 or f2 set                       */
    ST_FAULT,
    ST__COUNT
} arm_state_t;

typedef enum { DIR_NONE = 0, DIR_DEPLOY, DIR_STOW } move_dir_t;

typedef enum {
    FLT_NONE = 0, FLT_BOTH_LIMITS, FLT_WRONG_LIMIT, FLT_SWITCH_STUCK,
    FLT_STALL, FLT_WRONG_DIR, FLT_OVERTRAVEL, FLT_TIMEOUT, FLT_SAFETY, FLT__COUNT
} fault_t;

typedef enum { LOCK_UNKNOWN = 0, LOCK_LOCKED, LOCK_UNLOCKED, LOCK_LOCKING, LOCK_UNLOCKING, LOCK__COUNT } lock_state_t;

/* Safety-task trip (global, latched until reset) */
typedef enum { TRIP_NONE = 0, TRIP_MOTION_SILENT, TRIP_STATUS_SILENT } trip_t;

/* status_task -> motion_task requests */
typedef enum { REQ_FAULT = 0, REQ_POSITION_LOST, REQ_SAVE } req_kind_t;
typedef struct {
    uint8_t  kind;          /* req_kind_t                                     */
    uint8_t  arm;           /* arm_id_t                                       */
    uint8_t  fault;         /* fault_t for REQ_FAULT                          */
    uint32_t seq;           /* move_seq (REQ_FAULT) / idle_seq (others)       */
} req_t;

typedef enum {
    CMD_DEPLOY = 0, CMD_STOW, CMD_START, CMD_STOP, CMD_RESET,
    CMD_LOCK, CMD_UNLOCK, CMD_SPEED, CMD_TIMEOUT, CMD_STATUS, CMD__COUNT
} cmd_type_t;

typedef enum { SIDE_BOTH = 0, SIDE_PORT, SIDE_STBD } side_t;

/* Output routing for a message */
typedef enum {
    ROUTE_TEXT = 0,   /* text line on UART0 + UART1                           */
    ROUTE_JSON,       /* JSON line on UART0 + UART1                           */
    ROUTE_AUTO,       /* text on UART0, JSON on UART1                         */
} route_t;

typedef struct {
    cmd_type_t type;
    side_t     side;
    route_t    route;       /* ROUTE_TEXT or ROUTE_JSON                       */
    float      value;       /* CMD_SPEED: clamped speed; CMD_TIMEOUT: seconds */
    move_dir_t dir;         /* CMD_START: DIR_NONE = stopped direction        */
    float      time_s;      /* CMD_START: 0 = run to limit                    */
    float      speed;       /* CMD_START: 0 = default speed                   */
    uint8_t    err;         /* parse result: CMD_ERR_*                        */
} cmd_t;

enum { CMD_ERR_NONE = 0, CMD_ERR_TIME_RANGE, CMD_ERR_SPEED_RANGE };

typedef struct {
    uint8_t port;           /* uart_port_t it came from                       */
    char    text[RX_LINE_MAX];
} line_msg_t;

typedef struct {
    uint8_t route;          /* route_t                                        */
    uint8_t critical;       /* 1 = may use reserved slots                     */
    char    text[TX_TEXT_MAX];
    char    json[TX_JSON_MAX];
} evt_msg_t;

typedef struct {
    char line[LOG_LINE_MAX];
} log_msg_t;

typedef struct {
    arm_state_t state;
    move_dir_t  dir;            /* direction of current / last move           */
    fault_t     fault;
    route_t     route;          /* format for async events of this arm        */
    const char *move_cmd;       /* "deploy" / "stow" / "start"                */

    /* inputs */
    bool    raw_deploy, raw_stow;          /* raw (active = true)             */
    bool    lim_deploy, lim_stow;          /* debounced LS2 / LS1             */
    uint8_t deb_cnt_deploy, deb_cnt_stow;
    int     pcnt_prev;
    int32_t count;                         /* + = deployed side               */
    bool    dist_restored;
    bool    pos_valid;                     /* count referenced to stow = 0    */

    /* persisted in the NVS record (arm_rec_t) */
    uint8_t  f1, f2;                       /* deploy / stow stopped part-way; never both 1 */
    uint32_t time_used_ms;                 /* run time of the stopped move    */
    int32_t  stroke;                       /* learned deploy position (counts) */
    bool     stroke_valid;
    uint32_t rate_cps;                     /* counts/s at 100 % duty, 0 = unknown */
    uint32_t stop_time_ms;                 /* driven segment cut by last stop/fault */

    /* lock servo */
    bool     servo_err;                    /* last servo write failed         */

    /* idle drift (checked by status_task) */
    int32_t  idle_ref_count;               /* count when the motor went off  */
    uint32_t idle_seq;                     /* +1 each time the reference is set */

    /* current move leg */
    uint32_t window_ms;                    /* time allowed for this leg       */
    bool     timed;                        /* start cmd with time: window end = stop */
    bool     continuing;                   /* resumes the stopped move (keeps time_used) */
    int32_t  eta_ms;                       /* ETA at start, -1 = unknown      */
    float    full_duty_s;                  /* integral of duty/100 over the leg */

    /* motion */
    float   duty;                          /* % requested                     */
    float   target_duty;                   /* %                               */
    float   speed_override;                /* start cmd speed, 0 = s_speed    */
    bool    limit_hold;                    /* raw target seen, awaiting debounce */
    bool    drive_started;                 /* begin_drive() done for this move */
    int64_t drive_not_before_us;           /* dead-time gate                  */
    int64_t move_start_us;
    int32_t start_count;
    bool    start_switch_was_pressed;
    uint32_t move_seq;                     /* +1 per drive start (status checks) */
    int64_t last_motion_end_us;
    move_dir_t last_dir;

    /* 'both' group completion */
    uint32_t group_id;
    bool     group_done;
} arm_t;

/* Read-only copy of an arm, published by motion for the other tasks. */
typedef struct {
    arm_state_t  state;
    lock_state_t lock;
    fault_t      fault;
    move_dir_t   dir;
    route_t      route;
    const char  *move_cmd;
    int32_t      count;
    int32_t      stroke;                /* 0 = not learned                   */
    bool         stroke_valid;
    uint32_t     time_used_ms;
    uint32_t     rate_cps;
    bool         pos_valid;
    bool         servo_err;
    bool         ls1, ls2;              /* stow / deploy switch closed       */
    uint8_t      f1, f2;
    uint32_t     run_ms;
    uint32_t     leg_ms;                /* elapsed in this leg               */
    uint32_t     move_seq;
    int32_t      start_count;
    bool         start_was_pressed;
    bool         drive_started;
    bool         limit_hold;
    int32_t      idle_ref_count;
    uint32_t     idle_seq;
    uint32_t     window_ms;             /* window of this leg                */
    int32_t      eta_ms;                /* live ETA, -1 = unknown            */
    float        duty;
} arm_snap_t;

typedef struct {
    arm_snap_t arm[ARM_COUNT];
    float      speed;
    uint32_t   timeout_s;
} sys_snap_t;

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmpr;
    mcpwm_gen_handle_t   gen;
    bool                 forced_low;
    pcnt_unit_handle_t   pcnt;
} arm_hal_t;

/* One NVS record per arm ("<arm>_rec"). Explicit padding, zeroed before use,
 * so the CRC covers deterministic bytes. */
typedef struct {
    uint16_t version;          /* ARM_REC_VERSION                            */
    uint8_t  f1;               /* 0/1                                        */
    uint8_t  f2;               /* 0/1, never both 1                          */
    uint8_t  stroke_valid;     /* 0/1                                        */
    uint8_t  pos_valid;        /* 0/1: count is referenced to stow = 0       */
    uint8_t  pad[2];           /* 0                                          */
    uint32_t time_used_ms;
    int32_t  count;
    int32_t  stroke;
    uint32_t rate_cps;
    uint32_t stop_time_ms;
    uint32_t crc;              /* CRC32 of all bytes before this field       */
} arm_rec_t;
_Static_assert(sizeof(arm_rec_t) == 32U, "arm_rec_t layout (no implicit padding)");

/* =============================================================================
 * Static state — owned by motion_task unless noted
 * ========================================================================== */
static arm_t        s_arm[ARM_COUNT];
static arm_hal_t    s_hal[ARM_COUNT];               /* motor part: under s_motor_mtx */
static lock_state_t s_lock_state[ARM_COUNT] = { LOCK_UNKNOWN, LOCK_UNKNOWN };
static int64_t      s_lock_settle_until_us[ARM_COUNT];
static route_t      s_lock_route[ARM_COUNT] = { ROUTE_AUTO, ROUTE_AUTO };
static float        s_speed = DEFAULT_SPEED;        /* signed, clamped [-1,1] */
static float        s_speed_saved = DEFAULT_SPEED;
static uint32_t     s_group_seq;
static arm_rec_t    s_rec_saved[ARM_COUNT];         /* last record written / loaded */
static bool         s_rec_saved_ok[ARM_COUNT];
static int64_t      s_last_nvs_save_us;
static bool         s_hw_err_logged;

static nvs_handle_t s_nvs;
static bool         s_nvs_ok;

/* motion <-> safety (atomics) */
static _Atomic uint32_t s_timeout_s = MOVE_TIMEOUT_DEFAULT_S;  /* motion writes  */
static _Atomic uint32_t s_trip;                     /* trip_t: safety sets, reset clears */
static _Atomic bool     s_pwm_detached;             /* PWM pins forced to GPIO: reboot   */
static _Atomic uint32_t s_motion_beat;              /* motion: +1 per tick               */
static _Atomic uint32_t s_status_beat;              /* status: +1 per loop               */
static atomic_uint_fast32_t s_tx_drop;              /* any task                   */

/* Mutexes (static) */
static SemaphoreHandle_t s_motor_mtx, s_snap_mtx;
static StaticSemaphore_t s_motor_mtx_buf, s_snap_mtx_buf;
static sys_snap_t        s_snap;                    /* under s_snap_mtx           */

/* Written once in app_main before the ISR can matter; read by ISR. */
static TaskHandle_t volatile s_motion_task;

/* Queues (static storage) */
static QueueHandle_t s_line_q, s_cmd_q, s_req_q, s_evt_q, s_log_q;
static StaticQueue_t s_line_q_buf, s_cmd_q_buf, s_req_q_buf, s_evt_q_buf, s_log_q_buf;
static uint8_t s_line_q_store[LINE_QUEUE_LEN * sizeof(line_msg_t)];
static uint8_t s_cmd_q_store[CMD_QUEUE_LEN * sizeof(cmd_t)];
static uint8_t s_req_q_store[REQ_QUEUE_LEN * sizeof(req_t)];
static uint8_t s_evt_q_store[EVT_QUEUE_LEN * sizeof(evt_msg_t)];
static uint8_t s_log_q_store[LOG_QUEUE_LEN * sizeof(log_msg_t)];

/* Tasks (static storage) */
static StaticTask_t s_safety_tcb, s_motion_tcb, s_cmd_tcb, s_uart_tcb, s_status_tcb;
static StackType_t  s_safety_stack[SAFETY_STACK_BYTES];
static StackType_t  s_motion_stack[MOTION_STACK_BYTES];
static StackType_t  s_cmd_stack[CMD_STACK_BYTES];
static StackType_t  s_uart_stack[UART_STACK_BYTES];
static StackType_t  s_status_stack[STATUS_STACK_BYTES];
static TaskHandle_t s_task_h[5];

static vprintf_like_t s_prev_vprintf;               /* set once at boot       */

static const char *const STATE_NAME[ST__COUNT] = {
    "UNKNOWN", "STOWED", "DEPLOYED", "STARTING", "DEPLOYING",
    "STOWING", "STOPPED", "FAULT"
};
static const char *const CMD_NAME[CMD__COUNT] = {
    "deploy", "stow", "start", "stop", "reset", "lock", "unlock", "set_speed", "set_timeout", "status"
};
static const char *const FAULT_NAME[FLT__COUNT] = {
    "NONE", "BOTH_LIMITS", "WRONG_LIMIT", "SWITCH_STUCK",
    "STALL", "WRONG_DIR", "OVERTRAVEL", "TIMEOUT", "SAFETY"
};
static const char *const LOCK_NAME[LOCK__COUNT] = {
    "UNKNOWN", "LOCKED", "UNLOCKED", "LOCKING", "UNLOCKING"
};

/* =============================================================================
 * Small helpers
 * ========================================================================== */
static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline int64_t ms_to_us(uint32_t ms) { return (int64_t)ms * 1000LL; }

static inline bool arm_is_moving(const arm_t *a)
{
    return (a->state == ST_STARTING) || (a->state == ST_DEPLOYING) || (a->state == ST_STOWING);
}

static inline const char *dir_name(move_dir_t d)
{
    return (d == DIR_DEPLOY) ? "deploy" : (d == DIR_STOW) ? "stow" : "none";
}

static inline bool side_includes(side_t s, arm_id_t id)
{
    return (s == SIDE_BOTH) || ((s == SIDE_PORT) && (id == ARM_PORT)) || ((s == SIDE_STBD) && (id == ARM_STBD));
}

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
    }
}

static bool reset_reason_abnormal(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_PANIC: case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:   case ESP_RST_BROWNOUT:
        return true;
    default:
        return false;
    }
}

static void halt_safe(const char *why);

/* Each task subscribes ITSELF to the TWDT before its first reset. Adding it
 * from app_main after xTaskCreateStatic races: the new task (higher prio /
 * other core) runs first and esp_task_wdt_reset() fails "task not found". */
static void task_wdt_subscribe(void)
{
    if (esp_task_wdt_add(NULL) != ESP_OK) { halt_safe(pcTaskGetName(NULL)); }
}

/* =============================================================================
 * DIAG: TX queue producers (any task)                    [SOP §8.2, §9.2]
 * ========================================================================== */
static void tx_post(route_t route, bool critical, const char *text, const char *json)
{
    if (s_evt_q == NULL) { return; }
    /* Non-critical messages never use the reserved slots, so PROGRESS/logs
     * are dropped first and FAULT events always find space. */
    if (!critical && (uxQueueSpacesAvailable(s_evt_q) <= EVT_RESERVED_SLOTS)) {
        (void)atomic_fetch_add(&s_tx_drop, 1U);
        return;
    }
    evt_msg_t m;
    m.route    = (uint8_t)route;
    m.critical = critical ? 1U : 0U;
    (void)snprintf(m.text, sizeof m.text, "%s", (text != NULL) ? text : "");
    (void)snprintf(m.json, sizeof m.json, "%s", (json != NULL) ? json : "{}");
    if (xQueueSend(s_evt_q, &m, 0) != pdTRUE) {
        (void)atomic_fetch_add(&s_tx_drop, 1U);
    }
}

/* System event: text as given, JSON = {"evt","ok":false,"code","msg"} */
static void tx_simple(route_t route, bool critical, const char *evt, const char *code, const char *text)
{
    char j[TX_JSON_MAX];
    (void)snprintf(j, sizeof j, "{\"evt\":\"%s\",\"ok\":false,\"code\":\"%s\",\"msg\":\"%s\"}",
                   evt, code, text);
    tx_post(route, critical, text, j);
}

/* Reply code = the leading CODE token of msg ("POSITION_UNKNOWN: ..." ->
 * POSITION_UNKNOWN); messages without one ("Ack: ...") -> OK / ERROR. */
static void msg_code(const char *msg, bool ok, char *out, size_t cap)
{
    size_t n = 0U;
    while ((msg[n] != '\0') && (((msg[n] >= 'A') && (msg[n] <= 'Z')) || (msg[n] == '_') ||
                                ((msg[n] >= '0') && (msg[n] <= '9')))) {
        n++;
    }
    if ((n >= 2U) && ((msg[n] == '\0') || (msg[n] == ':') || (msg[n] == ' '))) {
        if (n >= cap) { n = cap - 1U; }
        (void)memcpy(out, msg, n);
        out[n] = '\0';
    } else {
        (void)snprintf(out, cap, "%s", ok ? "OK" : "ERROR");
    }
}

/* Console sharing: every ESP_LOG line becomes a "# ..." line via uart_task. */
static int log_vprintf_hook(const char *fmt, va_list args)
{
    if ((s_log_q == NULL) || xPortInIsrContext()) {
        return (s_prev_vprintf != NULL) ? s_prev_vprintf(fmt, args) : 0;
    }
    log_msg_t m;
    m.line[0] = '#';
    m.line[1] = ' ';
    int n = vsnprintf(&m.line[2], sizeof m.line - 2U, fmt, args);
    if (n < 0) { return 0; }
    size_t len = strnlen(m.line, sizeof m.line);
    while ((len > 2U) && ((m.line[len - 1U] == '\n') || (m.line[len - 1U] == '\r'))) {
        m.line[--len] = '\0';
    }
    if (len <= 2U) { return n; }   /* bare newline fragment */
    if (xQueueSend(s_log_q, &m, 0) != pdTRUE) {
        (void)atomic_fetch_add(&s_tx_drop, 1U);
    }
    return n;
}

/* =============================================================================
 * HAL: outputs (only place that touches MCPWM / GPIO outputs / LEDC)  [§5 / A]
 * ========================================================================== */
/* Boot step 1 and HALT path: drive PWM pins low as plain GPIO. */
static void hal_force_pwm_pins_low_gpio(void)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        /* level low first, then detach any MCPWM routing, then enable output */
        (void)gpio_set_level(ARM_HW[i].pwm_gpio, 0U);
        esp_rom_gpio_connect_out_signal((uint32_t)ARM_HW[i].pwm_gpio, SIG_GPIO_OUT_IDX, false, false);
        esp_rom_gpio_pad_select_gpio((uint32_t)ARM_HW[i].pwm_gpio);
        (void)gpio_set_direction(ARM_HW[i].pwm_gpio, GPIO_MODE_OUTPUT);
    }
}

static void hal_dir_pins_init(void)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        (void)gpio_reset_pin(ARM_HW[i].dir_gpio);
        (void)gpio_set_direction(ARM_HW[i].dir_gpio, GPIO_MODE_OUTPUT);
        (void)gpio_set_level(ARM_HW[i].dir_gpio, 0U);
    }
}

static void hal_note_error(const char *what, esp_err_t e)
{
    if (!s_hw_err_logged) {
        s_hw_err_logged = true;
        ESP_LOGE(TAG, "HAL error in %s: %s", what, esp_err_to_name(e));
    }
}

/* Caller must hold s_motor_mtx (motor_out / safety_force_low). */
static void hal_motor_write(arm_id_t id, float duty_pct)
{
    arm_hal_t *h = &s_hal[id];
    esp_err_t e;
    if (!(duty_pct > 0.0f)) {                 /* also catches NaN          */
        if (!h->forced_low) {
            e = mcpwm_generator_set_force_level(h->gen, 0, true);
            if (e != ESP_OK) { hal_note_error("force_low", e); }
            h->forced_low = true;
        }
        e = mcpwm_comparator_set_compare_value(h->cmpr, 0U);
        if (e != ESP_OK) { hal_note_error("cmp0", e); }
        return;
    }
    if (duty_pct > 100.0f) { duty_pct = 100.0f; }
    uint32_t ticks = (uint32_t)((duty_pct / 100.0f) * (float)MCPWM_PERIOD_TICKS);
    if (ticks >= MCPWM_PERIOD_TICKS) { ticks = MCPWM_PERIOD_TICKS - 1U; }
    e = mcpwm_comparator_set_compare_value(h->cmpr, ticks);
    if (e != ESP_OK) { hal_note_error("cmp", e); }
    if (h->forced_low) {
        e = mcpwm_generator_set_force_level(h->gen, -1, true);
        if (e != ESP_OK) { hal_note_error("release", e); }
        h->forced_low = false;
    }
}

static void hal_dir_write(arm_id_t id, uint32_t level)
{
    esp_err_t e = gpio_set_level(ARM_HW[id].dir_gpio, level);
    if (e != ESP_OK) { hal_note_error("dir", e); }
}

/* Returns true if the new pulse width was accepted by LEDC. */
static bool hal_servo_write_us(arm_id_t id, uint32_t pulse_us)
{
    const uint32_t full = (1UL << SERVO_LEDC_BITS);
    uint32_t duty = (uint32_t)(((uint64_t)pulse_us * full) / SERVO_PERIOD_US);
    esp_err_t e = ledc_set_duty(LEDC_LOW_SPEED_MODE, ARM_HW[id].servo_ch, duty);
    if (e == ESP_OK) { e = ledc_update_duty(LEDC_LOW_SPEED_MODE, ARM_HW[id].servo_ch); }
    if (e != ESP_OK) { hal_note_error("servo", e); }
    return (e == ESP_OK);
}

/* HAL: inputs (any task) */
static inline bool hal_lim_raw(gpio_num_t pin) { return gpio_get_level(pin) == 0; }

static int hal_enc_read(arm_id_t id)
{
    int v = 0;
    esp_err_t e = pcnt_unit_get_count(s_hal[id].pcnt, &v);
    if (e != ESP_OK) { hal_note_error("pcnt", e); }
    return v;
}

static void hal_enc_clear(arm_id_t id)
{
    esp_err_t e = pcnt_unit_clear_count(s_hal[id].pcnt);
    if (e != ESP_OK) { hal_note_error("pcnt_clr", e); }
}

/* =============================================================================
 * HAL: init
 * ========================================================================== */
static esp_err_t hal_uart_init(uart_port_t port, int tx, int rx)
{
    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t e = uart_driver_install(port, UART_RX_BUF_BYTES, UART_TX_BUF_BYTES, 0, NULL, 0);
    if (e == ESP_OK) { e = uart_param_config(port, &cfg); }
    if (e == ESP_OK) { e = uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); }
    /* uart_set_pin() leaves RX without a pull: an unconnected RX floats and
     * receives noise. Idle UART is high, so pull it up. */
    if ((e == ESP_OK) && (rx >= 0)) { e = gpio_pullup_en((gpio_num_t)rx); }
    if (e == ESP_OK) { e = uart_flush_input(port); }   /* drop boot-time junk */
    return e;
}

static esp_err_t hal_motor_init(arm_id_t id)
{
    arm_hal_t *h = &s_hal[id];
    const mcpwm_timer_config_t tcfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = (uint32_t)MCPWM_PERIOD_TICKS,
    };
    const mcpwm_operator_config_t ocfg = { .group_id = 0 };
    const mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
    const mcpwm_generator_config_t gcfg = { .gen_gpio_num = ARM_HW[id].pwm_gpio };

    esp_err_t e = mcpwm_new_timer(&tcfg, &h->timer);
    if (e == ESP_OK) { e = mcpwm_new_operator(&ocfg, &h->oper); }
    if (e == ESP_OK) { e = mcpwm_operator_connect_timer(h->oper, h->timer); }
    if (e == ESP_OK) { e = mcpwm_new_comparator(h->oper, &ccfg, &h->cmpr); }
    if (e == ESP_OK) { e = mcpwm_new_generator(h->oper, &gcfg, &h->gen); }
    if (e == ESP_OK) { e = mcpwm_comparator_set_compare_value(h->cmpr, 0U); }
    if (e == ESP_OK) {
        e = mcpwm_generator_set_action_on_timer_event(h->gen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    }
    if (e == ESP_OK) {
        e = mcpwm_generator_set_action_on_compare_event(h->gen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, h->cmpr, MCPWM_GEN_ACTION_LOW));
    }
    /* Hold output low (true 0 %) until a move releases it. */
    if (e == ESP_OK) { e = mcpwm_generator_set_force_level(h->gen, 0, true); h->forced_low = true; }
    if (e == ESP_OK) { e = mcpwm_timer_enable(h->timer); }
    if (e == ESP_OK) { e = mcpwm_timer_start_stop(h->timer, MCPWM_TIMER_START_NO_STOP); }
    return e;
}

/* GPIO19/20 carry encoder signals: detach the USB-Serial-JTAG PHY from the
 * pads and drop its D+/D- pulls, so they behave as plain GPIO inputs. */
static void hal_usb_pads_release(void)
{
    REG_SET_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PAD_PULL_OVERRIDE);
    REG_CLR_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_DP_PULLUP | USB_SERIAL_JTAG_DP_PULLDOWN |
                                           USB_SERIAL_JTAG_DM_PULLUP | USB_SERIAL_JTAG_DM_PULLDOWN);
    REG_CLR_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
}

static esp_err_t hal_encoder_init(arm_id_t id)
{
    arm_hal_t *h = &s_hal[id];
    const pcnt_unit_config_t ucfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    const pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = PCNT_GLITCH_NS };
    const pcnt_chan_config_t ca = { .edge_gpio_num = ARM_HW[id].enc_a_gpio, .level_gpio_num = ARM_HW[id].enc_b_gpio };
    const pcnt_chan_config_t cb = { .edge_gpio_num = ARM_HW[id].enc_b_gpio, .level_gpio_num = ARM_HW[id].enc_a_gpio };
    pcnt_channel_handle_t cha = NULL;
    pcnt_channel_handle_t chb = NULL;

    esp_err_t e = pcnt_new_unit(&ucfg, &h->pcnt);
    if (e == ESP_OK) { e = pcnt_unit_set_glitch_filter(h->pcnt, &fcfg); }
    if (e == ESP_OK) { e = pcnt_new_channel(h->pcnt, &ca, &cha); }
    if (e == ESP_OK) { e = pcnt_new_channel(h->pcnt, &cb, &chb); }
    if (e == ESP_OK) { e = pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE); }
    if (e == ESP_OK) { e = pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE); }
    if (e == ESP_OK) { e = pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE); }
    if (e == ESP_OK) { e = pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE); }
    /* accum_count needs the limits as watch points */
    if (e == ESP_OK) { e = pcnt_unit_add_watch_point(h->pcnt, PCNT_HIGH_LIMIT); }
    if (e == ESP_OK) { e = pcnt_unit_add_watch_point(h->pcnt, PCNT_LOW_LIMIT); }
    if (e == ESP_OK) { e = pcnt_unit_enable(h->pcnt); }
    if (e == ESP_OK) { e = pcnt_unit_clear_count(h->pcnt); }
    if (e == ESP_OK) { e = pcnt_unit_start(h->pcnt); }
    /* Internal pull-ups as backup to the external 4.7k pull-ups. */
    if (e == ESP_OK) { e = gpio_pullup_en(ARM_HW[id].enc_a_gpio); }
    if (e == ESP_OK) { e = gpio_pullup_en(ARM_HW[id].enc_b_gpio); }
    return e;
}

static void IRAM_ATTR limit_isr(void *arg)
{
    (void)arg;
    TaskHandle_t t = s_motion_task;
    if (t != NULL) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(t, &hpw);
        if (hpw == pdTRUE) { portYIELD_FROM_ISR(); }
    }
}

static esp_err_t hal_limits_init(void)
{
    uint64_t mask = 0ULL;
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        mask |= (1ULL << (uint32_t)ARM_HW[i].lim_deploy_gpio) | (1ULL << (uint32_t)ARM_HW[i].lim_stow_gpio);
    }
    const gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    esp_err_t e = gpio_config(&io);
    if (e == ESP_OK) { e = gpio_install_isr_service(0); }
    for (uint32_t i = 0U; (i < (uint32_t)ARM_COUNT) && (e == ESP_OK); i++) {
        e = gpio_isr_handler_add(ARM_HW[i].lim_deploy_gpio, limit_isr, NULL);
        if (e == ESP_OK) { e = gpio_isr_handler_add(ARM_HW[i].lim_stow_gpio, limit_isr, NULL); }
    }
    return e;
}

static esp_err_t hal_servo_init(void)
{
    const ledc_timer_config_t lt = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)SERVO_LEDC_BITS,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t e = ledc_timer_config(&lt);
    for (uint32_t i = 0U; (i < (uint32_t)ARM_COUNT) && (e == ESP_OK); i++) {
        const ledc_channel_config_t lc = {
            .gpio_num   = ARM_HW[i].servo_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = ARM_HW[i].servo_ch,
            .timer_sel  = LEDC_TIMER_0,    /* both servos share the 50 Hz timer */
            .duty       = 0,           /* no pulses -> servo not moved at boot */
            .hpoint     = 0,
        };
        e = ledc_channel_config(&lc);
    }
    return e;
}

/* =============================================================================
 * NVS (motion_task only, plus boot) — commit only on change/transition
 * ========================================================================== */
static const char *nvs_key(arm_id_t id, const char *suffix, char *buf, size_t cap)
{
    (void)snprintf(buf, cap, "%s_%s", ARM_HW[id].name, suffix);
    return buf;
}

static void nvs_commit_log(esp_err_t e, const char *key)
{
    if (e == ESP_OK) { e = nvs_commit(s_nvs); }
    if (e != ESP_OK) { ESP_LOGE(TAG, "NVS %s save failed: %s", key, esp_err_to_name(e)); }
}

static void nvs_save_speed(float v)
{
    if (!s_nvs_ok) { return; }
    esp_err_t e = nvs_set_blob(s_nvs, "saved_speed", &v, sizeof v);
    nvs_commit_log(e, "saved_speed");
    if (e == ESP_OK) { s_speed_saved = v; }
}

static void nvs_save_timeout(uint32_t sec)
{
    if (!s_nvs_ok) { return; }
    nvs_commit_log(nvs_set_u32(s_nvs, "timeout_s", sec), "timeout_s");
}

static uint32_t arm_rec_crc(const arm_rec_t *r)
{
    return esp_rom_crc32_le(0U, (const uint8_t *)r, (uint32_t)offsetof(arm_rec_t, crc));
}

/* While driving, the record is written "as if stopped now": flag for this
 * direction and time_used including the current leg, so a power loss
 * mid-move boots as STOPPED and can continue. */
static void arm_rec_build(arm_id_t id, arm_rec_t *r)
{
    const arm_t *a = &s_arm[id];
    (void)memset(r, 0, sizeof *r);
    r->version      = (uint16_t)ARM_REC_VERSION;
    r->f1           = a->f1;
    r->f2           = a->f2;
    r->stroke_valid = a->stroke_valid ? 1U : 0U;
    r->pos_valid    = a->pos_valid ? 1U : 0U;
    r->time_used_ms = (a->time_used_ms > TIME_MAX_MS) ? TIME_MAX_MS : a->time_used_ms;
    if (a->drive_started) {
        r->f1 = (a->dir == DIR_DEPLOY) ? 1U : 0U;
        r->f2 = (a->dir == DIR_STOW) ? 1U : 0U;
        uint32_t leg = (uint32_t)((now_us() - a->move_start_us) / 1000LL);
        uint32_t tot = a->time_used_ms + leg;
        r->time_used_ms = (tot > (MOVE_TIMEOUT_MAX_S * 1000U)) ? (MOVE_TIMEOUT_MAX_S * 1000U) : tot;
    }
    r->count        = a->count;
    r->stroke       = a->stroke_valid ? a->stroke : 0;
    r->rate_cps     = a->rate_cps;
    r->stop_time_ms = (a->stop_time_ms > TIME_MAX_MS) ? TIME_MAX_MS : a->stop_time_ms;
    r->crc          = arm_rec_crc(r);
}

/* Every field range-checked; any failure discards the whole record. */
static bool arm_rec_valid(const arm_rec_t *r)
{
    if (r->version != (uint16_t)ARM_REC_VERSION)                     { return false; }
    if (r->crc != arm_rec_crc(r))                                    { return false; }
    if ((r->f1 > 1U) || (r->f2 > 1U) || ((r->f1 + r->f2) > 1U))      { return false; }
    if ((r->stroke_valid > 1U) || (r->pos_valid > 1U))               { return false; }
    if ((r->pad[0] | r->pad[1]) != 0U)                               { return false; }
    if (r->time_used_ms > (MOVE_TIMEOUT_MAX_S * 1000U))              { return false; }
    if ((r->count < -(2 * MAX_STROKE_COUNTS)) || (r->count > (2 * MAX_STROKE_COUNTS))) { return false; }
    if (r->stroke_valid != 0U) {
        if ((r->stroke <= SWITCH_RELEASE_COUNTS) || (r->stroke > MAX_STROKE_COUNTS)) { return false; }
    } else if (r->stroke != 0) {
        return false;
    }
    if (r->rate_cps > RATE_MAX_CPS)                                  { return false; }
    if (r->stop_time_ms > (MOVE_TIMEOUT_MAX_S * 1000U))              { return false; }
    return true;
}

/* Arm record (f1, f2, time, count, stroke, rate, stop time) — written only
 * if it differs from the last record saved. */
static void nvs_save_arm(arm_id_t id)
{
    if (!s_nvs_ok) { return; }
    arm_rec_t r;
    arm_rec_build(id, &r);
    if (s_rec_saved_ok[id] && (memcmp(&r, &s_rec_saved[id], sizeof r) == 0)) { return; }
    char k[16];
    esp_err_t e = nvs_set_blob(s_nvs, nvs_key(id, "rec", k, sizeof k), &r, sizeof r);
    nvs_commit_log(e, k);
    if (e == ESP_OK) { s_rec_saved[id] = r; s_rec_saved_ok[id] = true; }
}

/* =============================================================================
 * Geometry / time helpers (motion data)
 * ========================================================================== */
static inline float arm_speed(const arm_t *a)
{
    return (a->speed_override > 0.0f) ? a->speed_override : fabsf(s_speed);
}

/* Forward overtravel limit: stroke + max(10 %, 0.5 turn); MAX_STROKE_COUNTS
 * stands in for the stroke until it is learned. */
static int32_t fwd_overtravel_limit(const arm_t *a)
{
    const int32_t st = a->stroke_valid ? a->stroke : MAX_STROKE_COUNTS;
    int32_t m = st / 10;
    if (m < HALF_TURN_COUNTS) { m = HALF_TURN_COUNTS; }
    return st + m;
}

/* Approach zone: max(10 % of stroke, 1 turn). */
static int32_t approach_zone(const arm_t *a)
{
    const int32_t z = a->stroke_valid ? (a->stroke / 10) : 0;
    return (z > ENC_CPR) ? z : ENC_CPR;
}

/* Remaining counts to the end of a move in dir, or -1 if not known. */
static int32_t remaining_counts(const arm_t *a, move_dir_t dir)
{
    if (!a->pos_valid) { return -1; }
    int32_t r;
    if (dir == DIR_DEPLOY) {
        if (!a->stroke_valid) { return -1; }
        r = a->stroke - a->count;
    } else {
        r = a->count;
    }
    return (r < 0) ? 0 : r;
}

/* ETA (ms) = remaining / (rate_cps x speed); -1 if pos/stroke/rate unknown. */
static int32_t arm_eta_ms(const arm_t *a, move_dir_t dir, float sp)
{
    if (!a->stroke_valid || (a->rate_cps == 0U) || !(sp > 0.0f)) { return -1; }
    const int32_t rem = remaining_counts(a, dir);
    if (rem < 0) { return -1; }
    const float ms = ((float)rem * 1000.0f) / ((float)a->rate_cps * sp);
    return (ms > 2.0e9f) ? INT32_MAX : (int32_t)ms;
}

/* =============================================================================
 * Snapshot + per-arm message (ICD)
 * ========================================================================== */
static int snap_pct(const arm_snap_t *s)
{
    if (!s->pos_valid || (s->stroke <= 0)) { return -1; }
    int32_t c = s->count;
    if (c < 0) { c = 0; }
    if (c > s->stroke) { c = s->stroke; }
    return (int)(((int64_t)c * 100) / s->stroke);
}

/* Per-arm reply / event. Every one carries the arm's full state, so the host
 * never has to poll after a reply. Usable from any task with a snapshot. */
static void ev_arm_snap(route_t route, bool critical, const char *evt, const char *cmd,
                        arm_id_t id, const arm_snap_t *s, bool ok, const char *msg)
{
    char t[TX_TEXT_MAX];
    char j[TX_JSON_MAX];
    char code[24];
    msg_code(msg, ok, code, sizeof code);
    const float pos = (float)s->count * METERS_PER_COUNT;
    const float tsec = (float)s->run_ms / 1000.0f;
    (void)snprintf(t, sizeof t, "%s side=%s state=%s lock=%s f1=%u f2=%u ls1=%d ls2=%d pos=%.3f m pct=%d t=%.1f s%s%s%s",
                   msg, ARM_HW[id].name, STATE_NAME[s->state], LOCK_NAME[s->lock],
                   (unsigned)s->f1, (unsigned)s->f2, s->ls1 ? 1 : 0, s->ls2 ? 1 : 0,
                   (double)pos, snap_pct(s), (double)tsec,
                   (s->fault != FLT_NONE) ? " fault=" : "", (s->fault != FLT_NONE) ? FAULT_NAME[s->fault] : "",
                   s->servo_err ? " SERVO_ERR" : "");
    (void)snprintf(j, sizeof j,
                   "{\"evt\":\"%s\",\"cmd\":\"%s\",\"side\":\"%s\",\"ok\":%s,\"code\":\"%s\",\"state\":\"%s\","
                   "\"lock\":\"%s\",\"f1\":%u,\"f2\":%u,\"ls1\":%d,\"ls2\":%d,\"pos_m\":%.3f,\"pct\":%d,"
                   "\"time_s\":%.1f,\"servo_err\":%d,\"fault\":\"%s\",\"msg\":\"%s\"}",
                   evt, cmd, ARM_HW[id].name, ok ? "true" : "false", code, STATE_NAME[s->state], LOCK_NAME[s->lock],
                   (unsigned)s->f1, (unsigned)s->f2, s->ls1 ? 1 : 0, s->ls2 ? 1 : 0,
                   (double)pos, snap_pct(s), (double)tsec, s->servo_err ? 1 : 0, FAULT_NAME[s->fault], msg);
    tx_post(route, critical, t, j);
}

/* Status per arm: f1, f2, pos_valid, count, %, stroke, time_used + state. */
static void ev_status_arm(route_t route, arm_id_t id, const arm_snap_t *s)
{
    char t[TX_TEXT_MAX];
    char j[TX_JSON_MAX];
    const float pos = (float)s->count * METERS_PER_COUNT;
    (void)snprintf(t, sizeof t,
                   "STATUS side=%s state=%s lock=%s f1=%u f2=%u pos_valid=%d count=%ld pct=%d stroke=%ld%s "
                   "time_used=%.1f s ls1=%d ls2=%d pos=%.3f m rate=%lu cps fault=%s%s",
                   ARM_HW[id].name, STATE_NAME[s->state], LOCK_NAME[s->lock], (unsigned)s->f1, (unsigned)s->f2,
                   s->pos_valid ? 1 : 0, (long)s->count, snap_pct(s), (long)s->stroke,
                   s->stroke_valid ? "" : "(none)", (double)s->time_used_ms / 1000.0,
                   s->ls1 ? 1 : 0, s->ls2 ? 1 : 0, (double)pos, (unsigned long)s->rate_cps,
                   FAULT_NAME[s->fault], s->servo_err ? " SERVO_ERR" : "");
    (void)snprintf(j, sizeof j,
                   "{\"evt\":\"status\",\"cmd\":\"status\",\"side\":\"%s\",\"ok\":%s,\"code\":\"STATUS\","
                   "\"state\":\"%s\",\"lock\":\"%s\",\"f1\":%u,\"f2\":%u,\"pos_valid\":%d,\"count\":%ld,"
                   "\"pct\":%d,\"stroke\":%ld,\"stroke_valid\":%d,\"time_used_s\":%.1f,\"ls1\":%d,\"ls2\":%d,"
                   "\"pos_m\":%.3f,\"rate_cps\":%lu,\"servo_err\":%d,\"fault\":\"%s\"}",
                   ARM_HW[id].name, (s->fault == FLT_NONE) ? "true" : "false", STATE_NAME[s->state],
                   LOCK_NAME[s->lock], (unsigned)s->f1, (unsigned)s->f2, s->pos_valid ? 1 : 0, (long)s->count,
                   snap_pct(s), (long)s->stroke, s->stroke_valid ? 1 : 0, (double)s->time_used_ms / 1000.0,
                   s->ls1 ? 1 : 0, s->ls2 ? 1 : 0, (double)pos, (unsigned long)s->rate_cps,
                   s->servo_err ? 1 : 0, FAULT_NAME[s->fault]);
    tx_post(route, false, t, j);
}

/* Progress: the per-arm fields plus duty, elapsed leg, window left, ETA. */
static void ev_progress_snap(arm_id_t id, const arm_snap_t *s)
{
    char t[TX_TEXT_MAX];
    char j[TX_JSON_MAX];
    const float pos   = (float)s->count * METERS_PER_COUNT;
    const float el    = (float)s->leg_ms / 1000.0f;
    const float left  = (s->window_ms > s->leg_ms) ? (float)(s->window_ms - s->leg_ms) / 1000.0f : 0.0f;
    const float eta   = (s->eta_ms >= 0) ? (float)s->eta_ms / 1000.0f : -1.0f;
    (void)snprintf(t, sizeof t, "PROG side=%s %s pos=%.3f m pct=%d duty=%.0f%% elapsed=%.1f s left=%.1f s eta=%.1f s f1=%u f2=%u",
                   ARM_HW[id].name, s->move_cmd, (double)pos, snap_pct(s), (double)s->duty, (double)el,
                   (double)left, (double)eta, (unsigned)s->f1, (unsigned)s->f2);
    (void)snprintf(j, sizeof j,
                   "{\"evt\":\"progress\",\"cmd\":\"%s\",\"side\":\"%s\",\"ok\":true,\"code\":\"PROGRESS\",\"state\":\"%s\","
                   "\"lock\":\"%s\",\"f1\":%u,\"f2\":%u,\"ls1\":%d,\"ls2\":%d,\"pos_m\":%.3f,\"pct\":%d,"
                   "\"duty\":%.0f,\"elapsed_s\":%.1f,\"window_left_s\":%.1f,\"eta_s\":%.1f,\"time_s\":%.1f}",
                   s->move_cmd, ARM_HW[id].name, STATE_NAME[s->state], LOCK_NAME[s->lock],
                   (unsigned)s->f1, (unsigned)s->f2, s->ls1 ? 1 : 0, s->ls2 ? 1 : 0, (double)pos, snap_pct(s),
                   (double)s->duty, (double)el, (double)left, (double)eta, (double)s->run_ms / 1000.0);
    tx_post(s->route, false, t, j);
}

static void ev_sys_status(route_t route, const sys_snap_t *ss)
{
    char t[TX_TEXT_MAX];
    char j[TX_JSON_MAX];
    const unsigned long drops = (unsigned long)atomic_load(&s_tx_drop);
    const unsigned long up = (unsigned long)(now_us() / 1000000LL);
    (void)snprintf(t, sizeof t, "STATUS sys fw=%s speed=%.2f timeout=%lus uptime=%lus txdrop=%lu%s",
                   FW_VERSION, (double)fabsf(ss->speed), (unsigned long)ss->timeout_s, up, drops,
                   SHUB_BENCH_TEST ? " BENCH_TEST" : "");
    (void)snprintf(j, sizeof j,
                   "{\"evt\":\"status\",\"side\":\"sys\",\"code\":\"STATUS\",\"fw\":\"%s\",\"speed\":%.2f,\"timeout_s\":%lu,"
                   "\"uptime_s\":%lu,\"txdrop\":%lu,\"bench\":%s}",
                   FW_VERSION, (double)fabsf(ss->speed), (unsigned long)ss->timeout_s, up, drops,
                   SHUB_BENCH_TEST ? "true" : "false");
    tx_post(route, false, t, j);
}

/* Full status (cmd_task reply, status_task heartbeat): one per arm + sys. */
static void status_report(route_t route, const sys_snap_t *ss)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        ev_status_arm(route, (arm_id_t)i, &ss->arm[i]);
    }
    ev_sys_status(route, ss);
}

/* ---- motion-side helpers ---- */
static void snap_fill(arm_id_t id, arm_snap_t *s, int64_t t)
{
    const arm_t *a = &s_arm[id];
    s->state      = a->state;
    s->lock       = s_lock_state[id];
    s->fault      = a->fault;
    s->dir        = a->dir;
    s->route      = a->route;
    s->move_cmd   = (a->move_cmd != NULL) ? a->move_cmd : dir_name(a->dir);
    s->count      = a->count;
    s->stroke     = a->stroke_valid ? a->stroke : 0;
    s->stroke_valid = a->stroke_valid;
    s->time_used_ms = a->time_used_ms;
    s->rate_cps   = a->rate_cps;
    s->pos_valid  = a->pos_valid;
    s->servo_err  = a->servo_err;
    s->ls1        = a->lim_stow;
    s->ls2        = a->lim_deploy;
    s->f1         = a->f1;
    s->f2         = a->f2;
    s->leg_ms     = a->drive_started ? (uint32_t)((t - a->move_start_us) / 1000LL) : 0U;
    s->move_seq   = a->move_seq;
    s->start_count = a->start_count;
    s->start_was_pressed = a->start_switch_was_pressed;
    s->drive_started = a->drive_started;
    s->limit_hold = a->limit_hold;
    s->idle_ref_count = a->idle_ref_count;
    s->idle_seq   = a->idle_seq;
    s->run_ms     = a->time_used_ms + s->leg_ms;
    s->window_ms  = a->window_ms;
    s->eta_ms     = arm_is_moving(a) ? arm_eta_ms(a, a->dir, arm_speed(a)) : -1;
    s->duty       = a->duty;
}

static void ev_arm(route_t route, bool critical, const char *evt, const char *cmd,
                   arm_id_t id, bool ok, const char *msg)
{
    arm_snap_t s;
    snap_fill(id, &s, now_us());
    ev_arm_snap(route, critical, evt, cmd, id, &s, ok, msg);
}

static void snap_publish(int64_t t)
{
    sys_snap_t ss;
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) { snap_fill((arm_id_t)i, &ss.arm[i], t); }
    ss.speed = s_speed;
    ss.timeout_s = atomic_load(&s_timeout_s);
    if (xSemaphoreTake(s_snap_mtx, pdMS_TO_TICKS(2)) == pdTRUE) {
        s_snap = ss;
        (void)xSemaphoreGive(s_snap_mtx);
    }
}

static bool snap_get(sys_snap_t *out)
{
    if (xSemaphoreTake(s_snap_mtx, pdMS_TO_TICKS(20)) != pdTRUE) { return false; }
    *out = s_snap;
    (void)xSemaphoreGive(s_snap_mtx);
    return true;
}

/* =============================================================================
 * MOTION: motor output (shared with safety_task through s_motor_mtx)
 * ========================================================================== */
static void motor_out(arm_id_t id, float duty, move_dir_t dir)
{
    if (xSemaphoreTake(s_motor_mtx, pdMS_TO_TICKS(5)) != pdTRUE) {
        hal_force_pwm_pins_low_gpio();          /* cannot happen; fail safe   */
        halt_safe("motor mutex");
    }
    (void)dir;
    if (atomic_load(&s_trip) != (uint32_t)TRIP_NONE) { duty = 0.0f; }   /* trip: all off */
    hal_motor_write(id, duty);
    (void)xSemaphoreGive(s_motor_mtx);
}

/* =============================================================================
 * MOTION: encoder reference, locks
 * ========================================================================== */
static void enc_zero(arm_id_t id)
{
    hal_enc_clear(id);
    s_arm[id].pcnt_prev     = 0;
    s_arm[id].count         = 0;
    s_arm[id].dist_restored = false;
    s_arm[id].pos_valid     = true;
}

/* Deploy limit = learned deploy position (if stow reference known). */
static void apply_deploy_reference(arm_id_t id)
{
    arm_t *a = &s_arm[id];
    if (a->pos_valid && !a->dist_restored) {
        if ((a->count > SWITCH_RELEASE_COUNTS) && (a->count <= MAX_STROKE_COUNTS) &&
            (!a->stroke_valid || (a->count != a->stroke))) {
            a->stroke = a->count;              /* saved with the arm record   */
            a->stroke_valid = true;
            ESP_LOGI(TAG, "%s stroke learned: %ld counts", ARM_HW[id].name, (long)a->count);
        }
    } else if (a->stroke_valid) {
        a->count = a->stroke;                 /* re-reference from the switch */
        a->pos_valid = true;
    } else {
        /* position unknown until first stow */
    }
}

static void lock_request(arm_id_t id, bool lock, route_t route)
{
    s_lock_route[id] = route;
    const bool ok = hal_servo_write_us(id, lock ? SERVO_LOCK_US : SERVO_UNLOCK_US);
    s_arm[id].servo_err = !ok;
    if (!ok) {
        /* Lock position not known: UNKNOWN is never UNLOCKED, so this arm's
         * motor stays off (lock rule). */
        s_lock_state[id] = LOCK_UNKNOWN;
        ev_arm(route, true, "fault", lock ? "lock" : "unlock", id, false, "SERVO_ERR: lock servo write failed");
        return;
    }
    s_lock_state[id] = lock ? LOCK_LOCKING : LOCK_UNLOCKING;
    s_lock_settle_until_us[id] = now_us() + ms_to_us(LOCK_SETTLE_MS);
}

static void lock_step(int64_t t)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        lock_state_t ls = s_lock_state[i];
        if (((ls == LOCK_LOCKING) || (ls == LOCK_UNLOCKING)) && (t >= s_lock_settle_until_us[i])) {
            bool locked = (ls == LOCK_LOCKING);
            s_lock_state[i] = locked ? LOCK_LOCKED : LOCK_UNLOCKED;
            ev_arm(s_lock_route[i], true, locked ? "locked" : "unlocked", locked ? "lock" : "unlock",
                   (arm_id_t)i, true, locked ? "LOCKED" : "UNLOCKED");
        }
    }
}

/* Lock follows position: set at DEPLOYED, open at STOWED. */
static void lock_align(arm_id_t id, route_t route)
{
    const lock_state_t ls = s_lock_state[id];
    if ((s_arm[id].state == ST_DEPLOYED) && (ls != LOCK_LOCKED) && (ls != LOCK_LOCKING)) {
        lock_request(id, true, route);
    } else if ((s_arm[id].state == ST_STOWED) && (ls != LOCK_UNLOCKED) && (ls != LOCK_UNLOCKING)) {
        lock_request(id, false, route);
    } else {
        /* already matches */
    }
}

/* State re-read from the switches; f1 / f2, count and pos_valid are kept
 * (reset, stop before the motor started). */
static void state_from_switches(arm_id_t id)
{
    arm_t *a = &s_arm[id];
    a->raw_deploy = hal_lim_raw(ARM_HW[id].lim_deploy_gpio);
    a->raw_stow   = hal_lim_raw(ARM_HW[id].lim_stow_gpio);
    if (a->lim_deploy && a->lim_stow) { a->state = ST_FAULT; a->fault = FLT_BOTH_LIMITS; }
    else if (a->lim_stow)             { a->state = ST_STOWED; }
    else if (a->lim_deploy)           { a->state = ST_DEPLOYED; }
    else                              { a->state = (a->f1 || a->f2) ? ST_STOPPED : ST_UNKNOWN; }
}

/* Boot only: state from the switches and the NVS record.
 *   LS1 closed : STOWED, count = 0, pos_valid = 1, f1 = f2 = time_used = 0
 *   LS2 closed : DEPLOYED, f1 = f2 = time_used = 0, count = stroke if
 *                stroke_valid, pos_valid = stroke_valid || record pos_valid
 *   both open  : record valid -> count/f1/f2 restored (boot_load_nvs),
 *                pos_valid = the record's pos_valid (1 unless the count was
 *                saved without a reference), STOPPED if f1 or f2 else UNKNOWN
 *                no record    -> pos_valid = 0, UNKNOWN
 *   both closed: FAULT BOTH_LIMITS */
static void boot_classify(arm_id_t id, bool rec_valid, bool rec_pos_valid)
{
    arm_t *a = &s_arm[id];
    if (a->lim_deploy && a->lim_stow) {
        a->state = ST_FAULT;
        a->fault = FLT_BOTH_LIMITS;
    } else if (a->lim_stow) {
        a->state = ST_STOWED;
        enc_zero(id);                         /* count = 0, pos_valid = 1 */
        a->f1 = 0U; a->f2 = 0U; a->time_used_ms = 0U;
    } else if (a->lim_deploy) {
        a->state = ST_DEPLOYED;
        a->f1 = 0U; a->f2 = 0U; a->time_used_ms = 0U;
        if (a->stroke_valid) { a->count = a->stroke; }
        a->pos_valid = a->stroke_valid || (rec_valid && rec_pos_valid);
    } else if (rec_valid) {
        a->pos_valid = rec_pos_valid;
        a->state = (a->f1 || a->f2) ? ST_STOPPED : ST_UNKNOWN;
    } else {
        a->pos_valid = false;
        a->state = ST_UNKNOWN;
    }
}

/* =============================================================================
 * MOTION: move primitives
 * ========================================================================== */
static void arm_stop_output(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];
    const bool was_driving = (a->duty > 0.0f) || a->drive_started;
    if (was_driving) {
        a->last_motion_end_us = t;
        a->last_dir = a->dir;
    }
    a->duty = 0.0f;
    a->target_duty = 0.0f;
    a->limit_hold = false;
    motor_out(id, 0.0f, DIR_NONE);   /* immediate, do not wait for end of tick */
    if (was_driving) {               /* new drift reference only after a drive */
        a->idle_ref_count = a->count;
        a->idle_seq++;
    }
}

/* Arm halted part-way while driving: remember direction (f1/f2) and run time. */
static void arm_mark_partway(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];
    if (!a->drive_started) { return; }
    uint32_t leg = (uint32_t)((t - a->move_start_us) / 1000LL);
    if (leg > TIME_MAX_MS) { leg = TIME_MAX_MS; }
    a->stop_time_ms = leg;
    a->time_used_ms = ((TIME_MAX_MS - a->time_used_ms) < leg) ? TIME_MAX_MS : (a->time_used_ms + leg);
    a->f1 = (a->dir == DIR_DEPLOY) ? 1U : 0U;
    a->f2 = (a->dir == DIR_STOW) ? 1U : 0U;
}

static void arm_fault(arm_id_t id, fault_t f, int64_t t)
{
    arm_t *a = &s_arm[id];
    arm_stop_output(id, t);
    arm_mark_partway(id, t);
    a->fault = f;
    a->state = ST_FAULT;
    a->drive_started = false;
    a->timed = false;
    a->group_id = 0U;
    nvs_save_arm(id);
    char msg[48];
    (void)snprintf(msg, sizeof msg, "FAULT: %s - send R", FAULT_NAME[f]);
    ev_arm(a->route, true, "fault", a->move_cmd, id, false, msg);
    if (f == FLT_TIMEOUT) {
        tx_simple(a->route, true, "failsafe", "FAILSAFE", "FAILSAFE: Timeout, motor stopped.");
    }
    ESP_LOGW(TAG, "%s FAULT %s", ARM_HW[id].name, FAULT_NAME[f]);
}

/* Stop (X) or start-time elapsed: halt, keep f1/f2 + time + position in NVS. */
static void arm_halt(arm_id_t id, int64_t t, route_t route, const char *cmd, const char *evt, bool ok,
                     const char *msg)
{
    arm_t *a = &s_arm[id];
    const bool was_moving = arm_is_moving(a);
    const bool drove = a->drive_started;
    arm_stop_output(id, t);
    if (was_moving) {
        if (drove) {
            arm_mark_partway(id, t);          /* f1/f2, time_used, stop_time */
            a->state = ST_STOPPED;
        } else {
            /* cancelled while waiting for the lock / reversal rest: keep
             * f1/f2, state from the switches as they read now */
            state_from_switches(id);
            lock_align(id, route);            /* re-lock if it is at LS2 */
        }
        a->drive_started  = false;
        a->timed          = false;
        a->speed_override = 0.0f;
        a->group_id       = 0U;
    }
    nvs_save_arm(id);
    ev_arm(route, true, evt, cmd, id, ok, msg);
}

/* Actually start driving (called when this arm's lock is UNLOCKED and dead-time passed). */
static void arm_begin_drive(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];
    const arm_hw_t *hw = &ARM_HW[id];
    uint32_t level = (a->dir == DIR_DEPLOY) ? hw->deploy_dir_level : (hw->deploy_dir_level ^ 1U);
    hal_dir_write(id, level);                 /* duty is 0 here                */
    a->state            = (a->dir == DIR_DEPLOY) ? ST_DEPLOYING : ST_STOWING;
    a->drive_started    = true;
    a->duty             = 0.0f;
    a->target_duty      = 0.0f;               /* set every tick by arm_step */
    a->limit_hold       = false;
    a->move_start_us    = t;
    a->start_count      = a->count;
    a->start_switch_was_pressed = (a->dir == DIR_DEPLOY) ? a->lim_stow : a->lim_deploy;
    a->move_seq++;                            /* status_task restarts its checks */
    if (!a->continuing) { a->time_used_ms = 0U; }   /* fresh move / reversal */
    a->full_duty_s        = 0.0f;
    ESP_LOGD(TAG, "%s drive %s DIR=%lu", hw->name, dir_name(a->dir), (unsigned long)level);
}

/* Accept a move: work out its window, unlock this arm's lock first if
 * needed, honour the 200 ms reversal rest. Returns false (and replies
 * WINDOW_EXHAUSTED) if less than WINDOW_MIN_MS would be allowed.
 *   continuing (forward & f1, reverse & f2): window = timeout - time_used
 *   fresh move or reversal                 : window = timeout, time_used = 0
 *   start command with a time              : window = that time          */
static bool arm_start_move(arm_id_t id, move_dir_t dir, route_t route, int64_t t,
                           const char *cmd, float speed_override, uint32_t time_ms)
{
    arm_t *a = &s_arm[id];
    const bool continuing = ((dir == DIR_DEPLOY) && (a->f1 != 0U)) || ((dir == DIR_STOW) && (a->f2 != 0U));
    const uint32_t timeout_ms = atomic_load(&s_timeout_s) * 1000U;
    uint32_t window;
    if (time_ms > 0U)     { window = time_ms; }
    else if (continuing)  { window = (timeout_ms > a->time_used_ms) ? (timeout_ms - a->time_used_ms) : 0U; }
    else                  { window = timeout_ms; }
    if (SHUB_BENCH_TEST && (time_ms == 0U) && (window < BENCH_WINDOW_MIN_MS)) { window = BENCH_WINDOW_MIN_MS; }
    if (window < WINDOW_MIN_MS) {
        char msg[80];
        (void)snprintf(msg, sizeof msg, "WINDOW_EXHAUSTED: %.1f s left of %lu s - reverse or set_timeout",
                       (double)window / 1000.0, (unsigned long)(timeout_ms / 1000U));
        ev_arm(route, true, "rejected", cmd, id, false, msg);
        return false;
    }
    a->continuing     = continuing;       /* time_used reset at drive start, so
                                           * a cancel before it keeps time_used */
    a->route          = route;
    a->dir            = dir;
    a->move_cmd       = cmd;
    a->speed_override = speed_override;
    a->timed          = (time_ms > 0U);
    a->window_ms      = window;
    a->eta_ms         = arm_eta_ms(a, dir, arm_speed(a));
    a->drive_started  = false;
    a->fault          = FLT_NONE;
    a->drive_not_before_us = t;
    if ((a->last_dir != DIR_NONE) && (a->last_dir != dir) &&
        ((t - a->last_motion_end_us) < ms_to_us(DIR_DEADTIME_MS))) {
        a->drive_not_before_us = a->last_motion_end_us + ms_to_us(DIR_DEADTIME_MS);
    }
    if ((s_lock_state[id] != LOCK_UNLOCKED) && (s_lock_state[id] != LOCK_UNLOCKING)) {
        lock_request(id, false, route);
    }
    a->state = ST_STARTING;
    return true;
}

/* Ack of an accepted start: window + ETA; WINDOW_SHORT if ETA > window. */
static void ack_started(const cmd_t *c, arm_id_t id, const char *what)
{
    const arm_t *a = &s_arm[id];
    char msg[112];
    if (a->eta_ms >= 0) {
        (void)snprintf(msg, sizeof msg, "Ack: %s speed %.2f window %.1f s ETA %.1f s", what,
                       (double)arm_speed(a), (double)a->window_ms / 1000.0, (double)a->eta_ms / 1000.0);
    } else {
        (void)snprintf(msg, sizeof msg, "Ack: %s speed %.2f window %.1f s ETA unknown", what,
                       (double)arm_speed(a), (double)a->window_ms / 1000.0);
    }
    ev_arm(c->route, false, "ack", CMD_NAME[c->type], id, true, msg);
    if ((a->eta_ms >= 0) && ((uint32_t)a->eta_ms > a->window_ms)) {
        (void)snprintf(msg, sizeof msg, "WINDOW_SHORT: ETA %.1f s > window %.1f s",
                       (double)a->eta_ms / 1000.0, (double)a->window_ms / 1000.0);
        ev_arm(c->route, true, "warn", CMD_NAME[c->type], id, false, msg);
    }
}

/* rate_cps from a completed leg: counts moved / (integral of duty/100 dt),
 * so ramp and approach slow-down do not bias it. Legs > 1 s only; never in
 * bench mode (hand-turned encoder). */
static void arm_learn_rate(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];
    if (SHUB_BENCH_TEST || ((t - a->move_start_us) <= ms_to_us(RATE_MIN_MOVE_MS)) || !(a->full_duty_s > 0.0f)) {
        return;
    }
    int32_t moved = a->count - a->start_count;
    if (moved < 0) { moved = -moved; }
    const float meas = (float)moved / a->full_duty_s;
    if (!(meas > 0.0f) || (meas > (float)RATE_MAX_CPS)) { return; }
    const int32_t old = (int32_t)a->rate_cps;
    const int32_t m = (int32_t)meas;
    a->rate_cps = (old == 0) ? (uint32_t)m : (uint32_t)(old + ((m - old) / RATE_EMA_DIV));
}

static void arm_complete(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];
    move_dir_t dir = a->dir;
    arm_stop_output(id, t);
    a->drive_started  = false;
    a->timed          = false;
    a->speed_override = 0.0f;
    a->state = (dir == DIR_DEPLOY) ? ST_DEPLOYED : ST_STOWED;
    arm_learn_rate(id, t);                    /* first: raw counts moved this leg */
    char msg[80];
    if (dir == DIR_DEPLOY) {
        /* LS2: check, then (re)learn the stroke from the encoder */
        if (a->pos_valid && a->stroke_valid && (labs((long)(a->count - a->stroke)) > (long)MISMATCH_COUNTS)) {
            (void)snprintf(msg, sizeof msg, "ENCODER_MISMATCH: count %ld vs stroke %ld at LS2",
                           (long)a->count, (long)a->stroke);
            ev_arm(a->route, true, "warn", a->move_cmd, id, false, msg);
        }
        if (a->pos_valid) {
            if ((a->count > SWITCH_RELEASE_COUNTS) && (a->count <= MAX_STROKE_COUNTS)) {
                a->stroke = a->count;
                a->stroke_valid = true;
            } else {
                ev_arm(a->route, true, "warn", a->move_cmd, id, false, "STROKE_OUT_OF_RANGE: not learned");
            }
        } else if (a->stroke_valid) {
            a->count = a->stroke;             /* re-reference from LS2 */
            a->pos_valid = true;
        } else {
            /* no reference yet: learned after the next stow + deploy */
        }
    } else {
        /* LS1: check, then position 0 */
        if (a->pos_valid && (labs((long)a->count) > (long)MISMATCH_COUNTS)) {
            (void)snprintf(msg, sizeof msg, "ENCODER_MISMATCH: count %ld at LS1", (long)a->count);
            ev_arm(a->route, true, "warn", a->move_cmd, id, false, msg);
        }
    }
    if (dir == DIR_STOW) { enc_zero(id); }    /* count = 0, pos_valid = 1   */
    a->idle_ref_count = a->count;             /* drift ref after re-reference */
    a->idle_seq++;
    a->f1 = 0U; a->f2 = 0U; a->time_used_ms = 0U;          /* limit reached */
    lock_align(id, a->route);                 /* lock at deploy, open at stow */
    nvs_save_arm(id);
    const char *up = (dir == DIR_DEPLOY) ? "DEPLOYED" : "STOWED";
    ev_arm(a->route, true, (dir == DIR_DEPLOY) ? "deployed" : "stowed", a->move_cmd, id, true, up);

    if (a->group_id != 0U) {
        a->group_done = true;
        arm_id_t other = (id == ARM_PORT) ? ARM_STBD : ARM_PORT;
        arm_t *o = &s_arm[other];
        if ((o->group_id == a->group_id) && o->group_done) {
            char t2[TX_TEXT_MAX];
            char j2[TX_JSON_MAX];
            (void)snprintf(t2, sizeof t2, "%s side=both", up);
            (void)snprintf(j2, sizeof j2,
                           "{\"evt\":\"%s\",\"cmd\":\"%s\",\"side\":\"both\",\"ok\":true,\"code\":\"%s\",\"msg\":\"%s\"}",
                           (dir == DIR_DEPLOY) ? "deployed" : "stowed", a->move_cmd, up, up);
            tx_post(a->route, true, t2, j2);
            a->group_id = 0U; o->group_id = 0U;
        }
    }
}

/* =============================================================================
 * MOTION: deploy / stow decision (spec rules, see header)
 * ========================================================================== */
typedef enum {
    DEC_START_CONT = 0,   /* own flag set: continue the stopped move         */
    DEC_START_FRESH,      /* new move                                        */
    DEC_LOCK_NOT_OPEN,    /* servo_err: lock position not known              */
    DEC_POS_UNKNOWN,      /* both switches open, no flag, no valid position  */
    DEC_AT_TARGET,        /* target switch already closed                    */
    DEC_SWITCH_FAULT,     /* LS1 and LS2 both closed                         */
    DEC_POS_CONFLICT      /* encoder says past the target end, switch open   */
} decision_t;

/* Safety-task trip (global): stops all motion until reset. */
static bool safety_tripped(void)
{
    return atomic_load(&s_trip) != (uint32_t)TRIP_NONE;
}

/* Deploy (dir = DIR_DEPLOY):            Stow (DIR_STOW) mirrors it:
 *   f1 == 1             -> start cont.    f2 == 1             -> start cont.
 *   servo_err           -> LOCK_NOT_OPEN  servo_err           -> LOCK_NOT_OPEN
 *   LS1 closed, LS2 open-> start fresh    LS2 closed, LS1 open-> start fresh
 *   both open: f1|f2|pos_valid -> start   both open: same
 *              else POSITION_UNKNOWN
 *   LS2 closed          -> ALREADY_DEPLOYED  LS1 closed       -> ALREADY_STOWED
 *   both closed         -> SWITCH_FAULT
 * Before any start: POSITION_CONFLICT if pos_valid and the count is past the
 * target end while the target switch is open (deploy: count > stroke +
 * max(10 %, 0.5 turn) with stroke_valid; stow: count < -0.5 turn). */
/* Encoder says the arm is already past the end it would drive to, while
 * that end's switch is open (deploy: count > stroke + max(10 %, 0.5 turn)
 * with stroke_valid; stow: count < -0.5 turn). Checked before any start. */
static bool pos_conflict(const arm_t *a, move_dir_t dir)
{
    if (!a->pos_valid) { return false; }
    if (dir == DIR_DEPLOY) {
        return a->stroke_valid && !a->lim_deploy && (a->count > fwd_overtravel_limit(a));
    }
    return !a->lim_stow && (a->count < -STOW_CONFLICT_COUNTS);
}

static decision_t decide_move(arm_id_t id, move_dir_t dir)
{
    const arm_t *a = &s_arm[id];
    const bool deploy    = (dir == DIR_DEPLOY);
    const bool own_flag  = deploy ? (a->f1 != 0U) : (a->f2 != 0U);
    const bool ls_start  = deploy ? a->lim_stow   : a->lim_deploy;
    const bool ls_target = deploy ? a->lim_deploy : a->lim_stow;
    decision_t d;

    if (own_flag)                        { d = DEC_START_CONT; }
    else if (a->servo_err)               { return DEC_LOCK_NOT_OPEN; }
    else if (ls_start && !ls_target)     { d = DEC_START_FRESH; }
    else if (!ls_start && !ls_target) {
        if ((a->f1 != 0U) || (a->f2 != 0U) || a->pos_valid) { d = DEC_START_FRESH; }
        else                                                { return DEC_POS_UNKNOWN; }
    }
    else if (ls_target && !ls_start)     { return DEC_AT_TARGET; }
    else                                 { return DEC_SWITCH_FAULT; }

    return pos_conflict(a, dir) ? DEC_POS_CONFLICT : d;
}

/* =============================================================================
 * MOTION: command handling
 * ========================================================================== */
static void mark_group(const bool accepted[ARM_COUNT])
{
    if (accepted[ARM_PORT] && accepted[ARM_STBD]) {
        s_group_seq++;
        if (s_group_seq == 0U) { s_group_seq = 1U; }
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            s_arm[i].group_id = s_group_seq;
            s_arm[i].group_done = false;
        }
    } else {
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            if (accepted[i]) { s_arm[i].group_id = 0U; s_arm[i].group_done = false; }
        }
    }
}

/* First checks for deploy / stow / start: fault latched or safety trip ->
 * reject; moving -> IN_PROGRESS (same dir) or BUSY. true = handled. */
static bool reject_if_busy(const cmd_t *c, arm_id_t id, move_dir_t dir)
{
    const arm_t *a = &s_arm[id];
    const char *cmd = CMD_NAME[c->type];
    char msg[64];
    if (a->state == ST_FAULT) {
        (void)snprintf(msg, sizeof msg, "FAULT_LATCHED: %s - send R", FAULT_NAME[a->fault]);
        ev_arm(c->route, true, "rejected", cmd, id, false, msg);
        return true;
    }
    if (safety_tripped()) {
        ev_arm(c->route, true, "rejected", cmd, id, false, "SAFETY_TRIP: send R");
        return true;
    }
    if (arm_is_moving(a)) {
        if (a->dir == dir) { ev_arm(c->route, false, "in_progress", cmd, id, true, "IN_PROGRESS"); }
        else               { ev_arm(c->route, true,  "busy", cmd, id, false, "BUSY: send stop first"); }
        return true;
    }
    return false;
}

static void report_at_target(const cmd_t *c, arm_id_t id, move_dir_t dir)
{
    arm_t *a = &s_arm[id];
    if (dir == DIR_DEPLOY) {
        a->state = ST_DEPLOYED;
        apply_deploy_reference(id);
    } else {
        a->state = ST_STOWED;
        enc_zero(id);
    }
    a->f1 = 0U; a->f2 = 0U; a->time_used_ms = 0U;
    a->idle_ref_count = a->count;             /* drift ref after re-reference */
    a->idle_seq++;
    lock_align(id, c->route);
    nvs_save_arm(id);
    ev_arm(c->route, false, (dir == DIR_DEPLOY) ? "deployed" : "stowed", CMD_NAME[c->type], id, true,
           (dir == DIR_DEPLOY) ? "ALREADY_DEPLOYED" : "ALREADY_STOWED");
}

static void handle_move_cmd(const cmd_t *c, move_dir_t dir, int64_t t)
{
    const char *cmd = CMD_NAME[c->type];
    const bool deploy = (dir == DIR_DEPLOY);
    bool accepted[ARM_COUNT] = { false, false };
    char msg[64];

    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_id_t id = (arm_id_t)i;
        arm_t *a = &s_arm[id];
        if (!side_includes(c->side, id) || reject_if_busy(c, id, dir)) { continue; }
        if (fabsf(s_speed) < MIN_SPEED) {
            ev_arm(c->route, true, "rejected", cmd, id, false, "SPEED_TOO_LOW");
            continue;
        }

        const decision_t d = decide_move(id, dir);
        switch (d) {
        case DEC_START_CONT:
        case DEC_START_FRESH:
            if (!arm_start_move(id, dir, c->route, t, cmd, 0.0f, 0U)) { break; }   /* WINDOW_EXHAUSTED */
            (void)snprintf(msg, sizeof msg, "%s (%s)", deploy ? "Deploy" : "Stow",
                           (d == DEC_START_CONT) ? "continue" : "fresh");
            ack_started(c, id, msg);
            accepted[i] = true;
            break;
        case DEC_LOCK_NOT_OPEN:
            ev_arm(c->route, true, "warn", cmd, id, false, "LOCK_NOT_OPEN: lock servo error - send U");
            break;
        case DEC_POS_UNKNOWN:
            ev_arm(c->route, true, "warn", cmd, id, false,
                   "POSITION_UNKNOWN: both switches open, no f1/f2 - use start with dir");
            break;
        case DEC_POS_CONFLICT:
            ev_arm(c->route, true, "rejected", cmd, id, false,
                   deploy ? "POSITION_CONFLICT: count past stroke, LS2 open"
                          : "POSITION_CONFLICT: count behind stow, LS1 open");
            break;
        case DEC_AT_TARGET:
            report_at_target(c, id, dir);
            break;
        case DEC_SWITCH_FAULT:
        default:
            a->route    = c->route;
            a->move_cmd = cmd;
            a->fault    = FLT_BOTH_LIMITS;
            a->state    = ST_FAULT;
            nvs_save_arm(id);
            ev_arm(c->route, true, "fault", cmd, id, false, "SWITCH_FAULT: LS1 and LS2 both closed");
            break;
        }
    }
    mark_group(accepted);
}

/* Start: resume a stopped move.
 *   f1 = f2 = 0                -> NOTHING_TO_RESUME
 *   no direction               -> forward if f1, reverse if f2
 *   target switch closed       -> ALREADY_DEPLOYED / ALREADY_STOWED
 *   time 1-180 s / speed 0.15-1.0 range-checked in cmd_task; a given speed
 *   applies to this move only. */
static void handle_start_cmd(const cmd_t *c, int64_t t)
{
    const char *cmd = CMD_NAME[c->type];
    bool accepted[ARM_COUNT] = { false, false };
    char msg[48];

    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_id_t id = (arm_id_t)i;
        if (!side_includes(c->side, id)) { continue; }
        arm_t *a = &s_arm[id];
        const move_dir_t dir = (c->dir != DIR_NONE) ? c->dir
                             : (a->f1 ? DIR_DEPLOY : (a->f2 ? DIR_STOW : DIR_NONE));
        if (reject_if_busy(c, id, dir)) { continue; }
        if ((a->f1 == 0U) && (a->f2 == 0U)) {
            ev_arm(c->route, true, "rejected", cmd, id, false, "NOTHING_TO_RESUME");
            continue;
        }
        if (a->lim_stow && a->lim_deploy) {
            a->route = c->route; a->move_cmd = cmd;
            a->fault = FLT_BOTH_LIMITS; a->state = ST_FAULT;
            nvs_save_arm(id);
            ev_arm(c->route, true, "fault", cmd, id, false, "SWITCH_FAULT: LS1 and LS2 both closed");
            continue;
        }
        if ((dir == DIR_DEPLOY) ? a->lim_deploy : a->lim_stow) { report_at_target(c, id, dir); continue; }
        if (pos_conflict(a, dir)) {
            ev_arm(c->route, true, "rejected", cmd, id, false,
                   (dir == DIR_DEPLOY) ? "POSITION_CONFLICT: count past stroke, LS2 open"
                                       : "POSITION_CONFLICT: count behind stow, LS1 open");
            continue;
        }
        if ((c->speed <= 0.0f) && (fabsf(s_speed) < MIN_SPEED)) {
            ev_arm(c->route, true, "rejected", cmd, id, false, "SPEED_TOO_LOW");
            continue;
        }
        const uint32_t tl = (c->time_s > 0.0f) ? (uint32_t)(c->time_s * 1000.0f) : 0U;
        if (!arm_start_move(id, dir, c->route, t, cmd, (c->speed > 0.0f) ? c->speed : 0.0f, tl)) { continue; }
        (void)snprintf(msg, sizeof msg, "Start %s%s", (dir == DIR_DEPLOY) ? "forward" : "reverse",
                       (tl > 0U) ? " (timed)" : "");
        ack_started(c, id, msg);
        accepted[i] = true;
    }
    mark_group(accepted);
}

static void handle_cmd(const cmd_t *c, int64_t t)
{
    switch (c->type) {
    case CMD_DEPLOY: handle_move_cmd(c, DIR_DEPLOY, t); break;
    case CMD_STOW:   handle_move_cmd(c, DIR_STOW, t);   break;
    case CMD_START:  handle_start_cmd(c, t);            break;

    case CMD_STOP:
        /* Software E-stop: f1/f2, run time and encoder position -> NVS. */
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            if (side_includes(c->side, (arm_id_t)i)) {
                arm_halt((arm_id_t)i, t, c->route, CMD_NAME[c->type], "stopped", true, "Ack: Stopped");
            }
        }
        break;

    case CMD_RESET: {
        /* a safety trip is cleared only here (any side) */
        const bool had_trip = safety_tripped();
        if (had_trip && atomic_load(&s_pwm_detached)) {
            tx_simple(c->route, true, "reset_denied", "RESET_DENIED", "RESET_DENIED: PWM forced off - reboot required");
            break;
        }
        if (had_trip) { atomic_store(&s_trip, (uint32_t)TRIP_NONE); }
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            arm_id_t id = (arm_id_t)i;
            arm_t *a = &s_arm[id];
            if (!side_includes(c->side, id)) { continue; }
            if (a->state != ST_FAULT) {
                if (had_trip) { ev_arm(c->route, false, "ack", CMD_NAME[c->type], id, true, "Ack: Reset (trip cleared)"); }
                continue;
            }
            if (a->lim_deploy && a->lim_stow) {
                ev_arm(c->route, true, "reset_denied", CMD_NAME[c->type], id, false, "RESET_DENIED: BOTH_LIMITS");
                continue;
            }
            a->fault = FLT_NONE;
            state_from_switches(id);          /* f1 / f2 kept: still resumable */
            lock_align(id, c->route);
            nvs_save_arm(id);
            ev_arm(c->route, false, "ack", CMD_NAME[c->type], id, true, "Ack: Reset");
        }
        break;
    }

    case CMD_SPEED: {
        float v = c->value;                       /* already finite & clamped */
        s_speed = v;
        if (fabsf(v - s_speed_saved) > SPEED_SAVE_EPS) { nvs_save_speed(v); }
        char t2[TX_TEXT_MAX];
        char j2[TX_JSON_MAX];
        (void)snprintf(t2, sizeof t2, "Ack: Speed saved at %.2f", (double)fabsf(v));
        (void)snprintf(j2, sizeof j2,
                       "{\"evt\":\"ack\",\"cmd\":\"set_speed\",\"ok\":true,\"code\":\"OK\",\"speed\":%.2f,\"msg\":\"%s\"}",
                       (double)fabsf(v), t2);
        tx_post(c->route, false, t2, j2);
        break;
    }

    case CMD_TIMEOUT: {
        const float v = c->value;
        const bool ok = isfinite(v) && (v >= (float)MOVE_TIMEOUT_MIN_S) && (v <= (float)MOVE_TIMEOUT_MAX_S);
        char t2[TX_TEXT_MAX];
        char j2[TX_JSON_MAX];
        if (ok) {
            uint32_t sec = (uint32_t)v;
            if (sec != atomic_load(&s_timeout_s)) { atomic_store(&s_timeout_s, sec); nvs_save_timeout(sec); }
            (void)snprintf(t2, sizeof t2, "Ack: Timeout saved at %lu s", (unsigned long)sec);
        } else {
            (void)snprintf(t2, sizeof t2, "TIMEOUT_OUT_OF_RANGE: must be %u..%u s",
                           (unsigned)MOVE_TIMEOUT_MIN_S, (unsigned)MOVE_TIMEOUT_MAX_S);
        }
        (void)snprintf(j2, sizeof j2,
                       "{\"evt\":\"%s\",\"cmd\":\"set_timeout\",\"ok\":%s,\"code\":\"%s\",\"timeout_s\":%lu,\"msg\":\"%s\"}",
                       ok ? "ack" : "rejected", ok ? "true" : "false", ok ? "OK" : "TIMEOUT_OUT_OF_RANGE",
                       (unsigned long)atomic_load(&s_timeout_s), t2);
        tx_post(c->route, !ok, t2, j2);
        break;
    }

    case CMD_LOCK:
        /* Each lock engages only at its own arm's deployed position. */
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            arm_id_t id = (arm_id_t)i;
            if (!side_includes(c->side, id)) { continue; }
            if (s_arm[id].state != ST_DEPLOYED) {
                ev_arm(c->route, true, "lock_denied", CMD_NAME[c->type], id, false, "LOCK_DENIED: not deployed");
            } else if (s_lock_state[id] == LOCK_LOCKED) {
                ev_arm(c->route, false, "locked", CMD_NAME[c->type], id, true, "LOCKED");
            } else {
                lock_request(id, true, c->route);
            }
        }
        break;

    case CMD_UNLOCK:
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            arm_id_t id = (arm_id_t)i;
            if (!side_includes(c->side, id)) { continue; }
            if (arm_is_moving(&s_arm[id])) {
                ev_arm(c->route, true, "unlock_denied", CMD_NAME[c->type], id, false, "UNLOCK_DENIED: moving");
            } else if (s_lock_state[id] == LOCK_UNLOCKED) {
                ev_arm(c->route, false, "unlocked", CMD_NAME[c->type], id, true, "UNLOCKED");
            } else {
                lock_request(id, false, c->route);
            }
        }
        break;

    case CMD_STATUS:                          /* handled by status_task */
    default:
        ESP_LOGW(TAG, "motion: unexpected cmd type %d", (int)c->type);
        break;
    }
}

/* =============================================================================
 * MOTION: per-arm step
 * ========================================================================== */
static void arm_step(arm_id_t id, int64_t t)
{
    arm_t *a = &s_arm[id];

    /* ---- idle arms (DRIFT / POSITION_LOST are detected by status_task) ---- */
    if (!arm_is_moving(a)) {
        a->duty = 0.0f;
        return;
    }

    /* ---- starting: wait for unlock settle + dead-time ---- */
    if (a->state == ST_STARTING) {
        a->duty = 0.0f;
        if (safety_tripped()) {               /* never begin a drive during a trip */
            arm_halt(id, t, a->route, a->move_cmd, "warn", false, "SAFETY_TRIP: start cancelled - send R");
            return;
        }
        if (a->servo_err && (s_lock_state[id] == LOCK_UNKNOWN)) {
            /* lock could not be opened: never drive against it */
            arm_halt(id, t, a->route, a->move_cmd, "warn", false, "LOCK_NOT_OPEN: servo error - move cancelled");
            return;
        }
        if ((s_lock_state[id] == LOCK_UNLOCKED) && (t >= a->drive_not_before_us)) {
            arm_begin_drive(id, t);
        }
        return;
    }

    /* ---- driving (DEPLOYING / STOWING) ---- */
    if (safety_tripped()) { arm_fault(id, FLT_SAFETY, t); return; }

    const bool raw_target = (a->dir == DIR_DEPLOY) ? a->raw_deploy : a->raw_stow;
    const bool deb_target = (a->dir == DIR_DEPLOY) ? a->lim_deploy : a->lim_stow;

    if (raw_target || a->limit_hold) {
        if (raw_target) {
            a->duty = 0.0f;                      /* fast stop, no waiting */
            a->limit_hold = true;
        }
        if (a->lim_deploy && a->lim_stow) { arm_fault(id, FLT_BOTH_LIMITS, t); return; }
        if (deb_target) { arm_complete(id, t); return; }
        if (!raw_target) {
            a->limit_hold = false;               /* bounce: resume normally */
        } else {
            return;                              /* hold PWM 0, check next tick */
        }
    }

    /* window: start-with-time -> stop like X; otherwise -> TIMEOUT fault */
    if ((t - a->move_start_us) >= ms_to_us(a->window_ms)) {
        if (a->timed) { arm_halt(id, t, a->route, a->move_cmd, "stopped", true, "TIME_ELAPSED"); }
        else          { arm_fault(id, FLT_TIMEOUT, t); }
        return;
    }

    /* STALL / WRONG_DIR / OVERTRAVEL / SWITCH_STUCK / WRONG_LIMIT /
     * BOTH_LIMITS run in status_task on the snapshot (REQ_FAULT). */

    /* approach slow-down: last max(10 % stroke, 1 turn) -> cap 25 % duty.
     * Forward needs stroke_valid, reverse needs pos_valid. */
    a->target_duty = arm_speed(a) * 100.0f;
    const int32_t rem = remaining_counts(a, a->dir);
    if ((rem >= 0) && (rem < approach_zone(a)) && (a->target_duty > APPROACH_DUTY_PCT)) {
        a->target_duty = APPROACH_DUTY_PCT;
    }

    /* all checks passed -> ramp toward target */
    const float step = 100.0f * (float)CONTROL_PERIOD_MS / (float)RAMP_TIME_MS;
    if (a->duty < a->target_duty) {
        a->duty += step;
        if (a->duty > a->target_duty) { a->duty = a->target_duty; }
    } else if (a->duty > a->target_duty) {
        a->duty -= step;
        if (a->duty < a->target_duty) { a->duty = a->target_duty; }
    } else {
        /* at target */
    }
}

/* =============================================================================
 * MOTION: inputs, tick, task
 * ========================================================================== */
static void debounce(bool raw, bool *deb, uint8_t *cnt)
{
    if (raw != *deb) {
        if (*cnt < 255U) { (*cnt)++; }
        if (*cnt >= DEBOUNCE_SAMPLES) { *deb = raw; *cnt = 0U; }
    } else {
        *cnt = 0U;
    }
}

static void read_inputs(void)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_id_t id = (arm_id_t)i;
        arm_t *a = &s_arm[id];
        a->raw_deploy = hal_lim_raw(ARM_HW[id].lim_deploy_gpio);
        a->raw_stow   = hal_lim_raw(ARM_HW[id].lim_stow_gpio);
        debounce(a->raw_deploy, &a->lim_deploy, &a->deb_cnt_deploy);
        debounce(a->raw_stow,   &a->lim_stow,   &a->deb_cnt_stow);
        int cur = hal_enc_read(id);
        int delta = cur - a->pcnt_prev;          /* accum_count: full int range */
        a->pcnt_prev = cur;
        a->count += (int32_t)delta * ARM_HW[id].enc_sign;
    }
}

/* Requests from status_task. A fault request applies only to the move it
 * was raised for (move_seq), so a stale request never hits a new move. */
static void handle_req(const req_t *r, int64_t t)
{
    if (r->arm >= (uint8_t)ARM_COUNT) { return; }
    const arm_id_t id = (arm_id_t)r->arm;
    arm_t *a = &s_arm[id];
    switch ((req_kind_t)r->kind) {
    case REQ_FAULT:
        if (((a->state == ST_DEPLOYING) || (a->state == ST_STOWING)) && (r->seq == a->move_seq) &&
            (r->fault < (uint8_t)FLT__COUNT)) {
            arm_fault(id, (fault_t)r->fault, t);
        } else if (!arm_is_moving(a) && (a->state != ST_FAULT) && (r->fault == (uint8_t)FLT_BOTH_LIMITS) &&
                   a->lim_deploy && a->lim_stow) {
            a->fault = FLT_BOTH_LIMITS;        /* both switches closed at rest */
            a->state = ST_FAULT;
            nvs_save_arm(id);
            ev_arm(a->route, true, "fault", "none", id, false, "SWITCH_FAULT: LS1 and LS2 both closed");
        } else {
            /* stale: that move already ended */
        }
        break;
    case REQ_POSITION_LOST:
        if (((a->state == ST_STOWED) && !a->lim_stow) || ((a->state == ST_DEPLOYED) && !a->lim_deploy)) {
            a->state = (a->f1 || a->f2) ? ST_STOPPED : ST_UNKNOWN;   /* pos_valid kept */
            nvs_save_arm(id);
        }
        break;
    case REQ_SAVE:
    default:
        if (!arm_is_moving(a)) { nvs_save_arm(id); }   /* DRIFT: keep stored position true */
        break;
    }
}

/* Limit ISR between ticks: raw target switch -> PWM 0 at once. */
static void fast_stop_check(void)
{
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_id_t id = (arm_id_t)i;
        arm_t *a = &s_arm[id];
        if (((a->state == ST_DEPLOYING) || (a->state == ST_STOWING)) && (a->duty > 0.0f)) {
            gpio_num_t pin = (a->dir == DIR_DEPLOY) ? ARM_HW[id].lim_deploy_gpio : ARM_HW[id].lim_stow_gpio;
            if (hal_lim_raw(pin)) {
                a->duty = 0.0f;
                a->limit_hold = true;
                motor_out(id, 0.0f, DIR_NONE);
            }
        }
    }
}

static void motion_tick(void)
{
    (void)esp_task_wdt_reset();
    (void)atomic_fetch_add(&s_motion_beat, 1U);
    const int64_t t = now_us();

    read_inputs();

    cmd_t c;
    while (xQueueReceive(s_cmd_q, &c, 0) == pdTRUE) {
        handle_cmd(&c, t);
    }
    req_t r;
    while (xQueueReceive(s_req_q, &r, 0) == pdTRUE) {
        handle_req(&r, t);
    }

    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_step((arm_id_t)i, t);
    }
    lock_step(t);

    /* outputs — interlock: no drive unless this arm's lock is UNLOCKED and
     * no safety trip is latched (checked in motor_out) */
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_t *a = &s_arm[i];
        if ((s_lock_state[i] != LOCK_UNLOCKED) || safety_tripped()) { a->duty = 0.0f; }
        motor_out((arm_id_t)i, a->duty, a->dir);
        if (a->drive_started && (a->duty > 0.0f)) {
            a->full_duty_s += (a->duty / 100.0f) * ((float)CONTROL_PERIOD_MS / 1000.0f);
        }
    }

    /* periodic position save while moving (stop/limit/fault save at once) */
    if ((t - s_last_nvs_save_us) >= ms_to_us(NVS_SAVE_PERIOD_MS)) {
        s_last_nvs_save_us = t;
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            if (s_arm[i].drive_started) { nvs_save_arm((arm_id_t)i); }   /* as if stopped now */
        }
    }
    snap_publish(t);
}

static void motion_task(void *pv)
{
    (void)pv;
    task_wdt_subscribe();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    TickType_t next_wake = xTaskGetTickCount() + period;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        int32_t remaining = (int32_t)(next_wake - now);
        if (remaining > 0) {
            if (ulTaskNotifyTake(pdTRUE, (TickType_t)remaining) > 0U) {
                fast_stop_check();             /* limit edge between ticks */
                continue;
            }
        }
        now = xTaskGetTickCount();
        if ((int32_t)(now - next_wake) > (int32_t)period) {
            next_wake = now;                   /* overrun: resync, no burst */
        }
        next_wake += period;
        motion_tick();
    }
}

/* =============================================================================
 * SAFETY task — alive-counter supervisor (100 ms)
 *   motion_task silent 500 ms -> both PWMs forced low + trip
 *   status_task silent 1 s    -> trip (motion then stops every arm)
 *   A trip stops all motion and is cleared only by reset (R).
 * ========================================================================== */
static void safety_force_low(void)
{
    if (xSemaphoreTake(s_motor_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) { hal_motor_write((arm_id_t)i, 0.0f); }
        (void)xSemaphoreGive(s_motor_mtx);
    } else {
        /* motion is stuck holding the motor mutex: detach MCPWM, pins low.
         * Only a reboot reconnects the PWM. */
        hal_force_pwm_pins_low_gpio();
        atomic_store(&s_pwm_detached, true);
    }
}

static void safety_trip(trip_t why, const char *msg)
{
    uint32_t none = (uint32_t)TRIP_NONE;
    if (atomic_compare_exchange_strong(&s_trip, &none, (uint32_t)why)) {
        tx_simple(ROUTE_AUTO, true, "failsafe", "SAFETY_TRIP", msg);
        ESP_LOGE(TAG, "%s", msg);
    }
}

static void safety_task(void *pv)
{
    (void)pv;
    task_wdt_subscribe();
    uint32_t m_last = atomic_load(&s_motion_beat);
    uint32_t s_last = atomic_load(&s_status_beat);
    int64_t  m_seen = now_us();
    int64_t  s_seen = now_us();
    int64_t  prev_loop = now_us();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SAFETY_PERIOD_MS));
        (void)esp_task_wdt_reset();
        const int64_t t = now_us();

        /* this task was itself held off (flash write etc.): re-baseline */
        if ((t - prev_loop) > ms_to_us(3U * SAFETY_PERIOD_MS)) { m_seen = t; s_seen = t; }
        prev_loop = t;

        const uint32_t mb = atomic_load(&s_motion_beat);
        const uint32_t sb = atomic_load(&s_status_beat);
        if (mb != m_last) { m_last = mb; m_seen = t; }
        if (sb != s_last) { s_last = sb; s_seen = t; }

        if ((t - m_seen) > ms_to_us(MOTION_SILENT_MS)) {
            safety_force_low();
            safety_trip(TRIP_MOTION_SILENT, "SAFETY_TRIP: motion task silent 500 ms - motors forced off");
        }
        if ((t - s_seen) > ms_to_us(STATUS_SILENT_MS)) {
            safety_trip(TRIP_STATUS_SILENT, "SAFETY_TRIP: status task silent 1 s - motion stopped");
        }
    }
}

/* =============================================================================
 * PROTOCOL: fixed-buffer flat JSON parser (no malloc)       [SOP §6, §8.1]
 * ========================================================================== */
#define JSON_MAX_PAIRS  6
#define JSON_KEY_MAX    16
#define JSON_STR_MAX    16

typedef enum { JV_STR = 0, JV_NUM, JV_BOOL, JV_NULL } jv_type_t;
typedef struct {
    char      key[JSON_KEY_MAX];
    jv_type_t type;
    char      str[JSON_STR_MAX];
    float     num;
} json_pair_t;

static const char *skip_ws(const char *p)
{
    while ((*p == ' ') || (*p == '\t') || (*p == '\r') || (*p == '\n')) { p++; }
    return p;
}

static const char *parse_jstring(const char *p, char *out, size_t cap)
{
    if (*p != '"') { return NULL; }
    p++;
    size_t n = 0U;
    while ((*p != '\0') && (*p != '"')) {
        char ch = *p;
        if (ch == '\\') {
            p++;
            ch = *p;
            if ((ch != '"') && (ch != '\\') && (ch != '/')) { return NULL; }
        }
        if ((n + 1U) >= cap) { return NULL; }
        out[n++] = ch;
        p++;
    }
    if (*p != '"') { return NULL; }
    out[n] = '\0';
    return p + 1;
}

/* returns number of pairs, or -1 on malformed input */
static int json_parse_flat(const char *s, json_pair_t *pairs, int max)
{
    const char *p = skip_ws(s);
    if (*p != '{') { return -1; }
    p = skip_ws(p + 1);
    if (*p == '}') { return (*skip_ws(p + 1) == '\0') ? 0 : -1; }
    int n = 0;
    for (;;) {
        if (n >= max) { return -1; }
        json_pair_t *jp = &pairs[n];
        (void)memset(jp, 0, sizeof *jp);
        p = parse_jstring(p, jp->key, sizeof jp->key);
        if (p == NULL) { return -1; }
        p = skip_ws(p);
        if (*p != ':') { return -1; }
        p = skip_ws(p + 1);
        if (*p == '"') {
            jp->type = JV_STR;
            p = parse_jstring(p, jp->str, sizeof jp->str);
            if (p == NULL) { return -1; }
        } else if ((*p == '-') || ((*p >= '0') && (*p <= '9'))) {
            char *end = NULL;
            float v = strtof(p, &end);
            if ((end == NULL) || (end == p)) { return -1; }
            jp->type = JV_NUM;
            jp->num = v;
            p = end;
        } else if (strncmp(p, "true", 4) == 0)  { jp->type = JV_BOOL; jp->num = 1.0f; p += 4; }
        else if (strncmp(p, "false", 5) == 0)   { jp->type = JV_BOOL; jp->num = 0.0f; p += 5; }
        else if (strncmp(p, "null", 4) == 0)    { jp->type = JV_NULL; p += 4; }
        else { return -1; }
        n++;
        p = skip_ws(p);
        if (*p == ',') { p = skip_ws(p + 1); continue; }
        if (*p == '}') { p = skip_ws(p + 1); return (*p == '\0') ? n : -1; }
        return -1;
    }
}

static const json_pair_t *json_find(const json_pair_t *pairs, int n, const char *key)
{
    for (int i = 0; i < n; i++) {
        if (strcmp(pairs[i].key, key) == 0) { return &pairs[i]; }
    }
    return NULL;
}

static bool clamp_speed(float v, float *out)
{
    if (!isfinite(v)) { return false; }
    if (v < -1.0f) { v = -1.0f; }
    if (v >  1.0f) { v =  1.0f; }
    *out = v;
    return true;
}

static bool parse_side_str(const char *s, side_t *out)
{
    if (strcmp(s, "both") == 0)                                   { *out = SIDE_BOTH; return true; }
    if (strcmp(s, "port") == 0)                                   { *out = SIDE_PORT; return true; }
    if ((strcmp(s, "stbd") == 0) || (strcmp(s, "starboard") == 0)) { *out = SIDE_STBD; return true; }
    return false;
}

/* start time / speed range (shared by letter and JSON forms); 0 = not given */
static uint8_t start_args_err(bool has_time, float time_s, bool has_speed, float speed)
{
    if (has_time && (!isfinite(time_s) || (time_s < START_TIME_MIN_S) || (time_s > START_TIME_MAX_S))) {
        return CMD_ERR_TIME_RANGE;
    }
    if (has_speed && (!isfinite(speed) || (speed < START_SPEED_MIN) || (speed > START_SPEED_MAX))) {
        return CMD_ERR_SPEED_RANGE;
    }
    return CMD_ERR_NONE;
}

static const cmd_t CMD_INIT = {
    .type = CMD_STATUS, .side = SIDE_BOTH, .route = ROUTE_TEXT,
    .value = 0.0f, .dir = DIR_NONE, .time_s = 0.0f, .speed = 0.0f, .err = CMD_ERR_NONE
};

/* JSON -> up to 2 commands (speed first). Returns count, 0 = invalid. */
static int parse_json_cmd(const char *line, cmd_t out[2])
{
    json_pair_t pairs[JSON_MAX_PAIRS];
    int np = json_parse_flat(line, pairs, JSON_MAX_PAIRS);
    if (np <= 0) { return 0; }
    int n = 0;

    /* legacy "set_speed" first (old behaviour: speed before command) */
    const json_pair_t *sp = json_find(pairs, np, "set_speed");
    if (sp != NULL) {
        float v;
        if ((sp->type != JV_NUM) || !clamp_speed(sp->num, &v)) { return 0; }
        out[n] = CMD_INIT;
        out[n].type = CMD_SPEED; out[n].value = v; out[n].route = ROUTE_JSON;
        n++;
    }

    const json_pair_t *cmd = json_find(pairs, np, "cmd");
    const json_pair_t *legacy = json_find(pairs, np, "command");
    if ((cmd != NULL) && (cmd->type == JV_STR)) {
        cmd_t c = CMD_INIT;
        c.route = ROUTE_JSON;
        const json_pair_t *side = json_find(pairs, np, "side");
        if (side != NULL) {
            if ((side->type != JV_STR) || !parse_side_str(side->str, &c.side)) { return 0; }
        }
        if      (strcmp(cmd->str, "deploy") == 0) { c.type = CMD_DEPLOY; }
        else if (strcmp(cmd->str, "stow") == 0)   { c.type = CMD_STOW; }
        else if (strcmp(cmd->str, "stop") == 0)   { c.type = CMD_STOP; }
        else if (strcmp(cmd->str, "reset") == 0)  { c.type = CMD_RESET; }
        else if (strcmp(cmd->str, "lock") == 0)   { c.type = CMD_LOCK; }
        else if (strcmp(cmd->str, "unlock") == 0) { c.type = CMD_UNLOCK; }
        else if (strcmp(cmd->str, "status") == 0) { c.type = CMD_STATUS; }
        else if (strcmp(cmd->str, "start") == 0) {
            c.type = CMD_START;
            const json_pair_t *d = json_find(pairs, np, "direction");
            const json_pair_t *tm = json_find(pairs, np, "time");
            const json_pair_t *spd = json_find(pairs, np, "speed");
            if (d != NULL) {
                if (d->type != JV_STR) { return 0; }
                if (strcmp(d->str, "forward") == 0)      { c.dir = DIR_DEPLOY; }
                else if (strcmp(d->str, "reverse") == 0) { c.dir = DIR_STOW; }
                else { return 0; }
            }
            if ((tm != NULL) && (tm->type != JV_NUM))   { return 0; }
            if ((spd != NULL) && (spd->type != JV_NUM)) { return 0; }
            if (tm != NULL)  { c.time_s = tm->num; }
            if (spd != NULL) { c.speed = spd->num; }
            c.err = start_args_err(tm != NULL, c.time_s, spd != NULL, c.speed);
        } else if (strcmp(cmd->str, "set_speed") == 0) {
            const json_pair_t *val = json_find(pairs, np, "value");
            if ((val == NULL) || (val->type != JV_NUM) || !clamp_speed(val->num, &c.value)) { return 0; }
            c.type = CMD_SPEED;
        } else if (strcmp(cmd->str, "set_timeout") == 0) {
            const json_pair_t *val = json_find(pairs, np, "value");
            if ((val == NULL) || (val->type != JV_NUM) || !isfinite(val->num)) { return 0; }
            c.type = CMD_TIMEOUT;
            c.value = val->num;               /* range-checked in motion */
        } else { return 0; }
        out[n++] = c;
    } else if ((legacy != NULL) && (legacy->type == JV_STR)) {
        cmd_t c = CMD_INIT;
        c.route = ROUTE_JSON;
        if      (strcmp(legacy->str, "Deploy") == 0)  { c.type = CMD_DEPLOY; }
        else if (strcmp(legacy->str, "Retract") == 0) { c.type = CMD_STOW; }
        else if (strcmp(legacy->str, "Pause") == 0)   { c.type = CMD_STOP; }  /* no pause: halts */
        else { return 0; }
        out[n++] = c;
    } else if (cmd != NULL || legacy != NULL) {
        return 0;                                     /* wrong value type */
    } else {
        /* speed-only message is valid */
    }
    return n;
}

/* Letter command -> 1 command. Returns 1, or 0 = invalid. */
static int parse_letter_cmd(const char *line, cmd_t *out)
{
    size_t len = strlen(line);
    if (len == 0U) { return 0; }
    char c0 = (char)toupper((unsigned char)line[0]);
    cmd_t c = CMD_INIT;

    if ((c0 == 'D') || (c0 == 'S') || (c0 == 'X') || (c0 == 'R') || (c0 == 'L') || (c0 == 'U')) {
        c.type = (c0 == 'D') ? CMD_DEPLOY : (c0 == 'S') ? CMD_STOW : (c0 == 'X') ? CMD_STOP :
                 (c0 == 'R') ? CMD_RESET  : (c0 == 'L') ? CMD_LOCK : CMD_UNLOCK;
        if (len == 1U)                         { c.side = SIDE_BOTH; }
        else if ((len == 2U) && (line[1] == '1')) { c.side = SIDE_PORT; }
        else if ((len == 2U) && (line[1] == '2')) { c.side = SIDE_STBD; }
        else { return 0; }
    } else if (c0 == 'G') {
        /* G, G1, G2 [+ optional D|S, ,<sec>, ,<speed %>] */
        size_t k = 1U;
        c.type = CMD_START;
        if (line[k] == '1')      { c.side = SIDE_PORT; k++; }
        else if (line[k] == '2') { c.side = SIDE_STBD; k++; }
        else                     { c.side = SIDE_BOTH; }
        const char d = (char)toupper((unsigned char)line[k]);
        if (d == 'D')      { c.dir = DIR_DEPLOY; k++; }
        else if (d == 'S') { c.dir = DIR_STOW; k++; }
        else               { /* stopped direction */ }
        bool has_t = false;
        bool has_s = false;
        if (line[k] == ',') {
            char *end = NULL;
            c.time_s = strtof(&line[k + 1U], &end);
            if ((end == NULL) || (end == &line[k + 1U])) { return 0; }
            has_t = true;
            k = (size_t)(end - line);
            if (line[k] == ',') {
                float pct = strtof(&line[k + 1U], &end);
                if ((end == NULL) || (end == &line[k + 1U])) { return 0; }
                c.speed = pct / 100.0f;
                has_s = true;
                k = (size_t)(end - line);
            }
        }
        if (line[k] != '\0') { return 0; }
        c.err = start_args_err(has_t, c.time_s, has_s, c.speed);
    } else if (c0 == 'V') {
        char *end = NULL;
        float pct = strtof(&line[1], &end);
        if ((len < 2U) || (end == NULL) || (end == &line[1]) || (*end != '\0')) { return 0; }
        if (!isfinite(pct) || (pct < 0.0f) || (pct > 100.0f)) { return 0; }
        c.type = CMD_SPEED;
        c.value = pct / 100.0f;
    } else if (c0 == 'T') {
        char *end = NULL;
        long sec = strtol(&line[1], &end, 10);
        if ((len < 2U) || (end == NULL) || (end == &line[1]) || (*end != '\0')) { return 0; }
        c.type = CMD_TIMEOUT;
        c.value = (float)sec;                 /* range-checked in motion */
    } else if ((c0 == '?') && (len == 1U)) {
        c.type = CMD_STATUS;
    } else {
        return 0;
    }
    *out = c;
    return 1;
}

/* =============================================================================
 * COMMAND task — line -> parsed / validated command -> motion or status
 * ========================================================================== */
static void cmd_task(void *pv)
{
    (void)pv;
    static line_msg_t lm;                              /* only this task      */
    task_wdt_subscribe();
    for (;;) {
        (void)esp_task_wdt_reset();
        if (xQueueReceive(s_line_q, &lm, pdMS_TO_TICKS(500)) != pdTRUE) { continue; }

        const char *p = lm.text;
        cmd_t cmds[2];
        int n;
        route_t fmt;
        if (p[0] == '{') { n = parse_json_cmd(p, cmds); fmt = ROUTE_JSON; }
        else             { n = parse_letter_cmd(p, &cmds[0]); fmt = ROUTE_TEXT; }

        if (n <= 0) {
            tx_simple(fmt, false, "error", "UNKNOWN_COMMAND", "Unknown command");
            ESP_LOGW(TAG, "uart%u unknown: %.40s", (unsigned)lm.port, p);
            continue;
        }
        for (int i = 0; i < n; i++) {
            if (cmds[i].type == CMD_STATUS) {         /* answered here from the snapshot */
                static sys_snap_t ss;
                if (snap_get(&ss)) { status_report(cmds[i].route, &ss); }
                continue;
            }
            if (cmds[i].err != CMD_ERR_NONE) {
                const char *code = (cmds[i].err == CMD_ERR_TIME_RANGE) ? "TIME_OUT_OF_RANGE" : "SPEED_OUT_OF_RANGE";
                char j[TX_JSON_MAX];
                (void)snprintf(j, sizeof j,
                               "{\"evt\":\"rejected\",\"cmd\":\"%s\",\"ok\":false,\"code\":\"%s\",\"msg\":\"%s\"}",
                               CMD_NAME[cmds[i].type], code, code);
                tx_post(fmt, true, code, j);
                continue;
            }
            if (xQueueSend(s_cmd_q, &cmds[i], pdMS_TO_TICKS(10)) != pdTRUE) { tx_simple(fmt, true, "busy", "BUSY_QUEUE", "BUSY_QUEUE"); }
        }
    }
}

/* =============================================================================
 * UART task — RX framing for both UARTs + the only writer of both UARTs
 * ========================================================================== */
typedef struct {
    uart_port_t port;
    bool        console;          /* UART0: echo, no idle line end           */
    char        buf[RX_LINE_MAX];
    size_t      len;
    bool        overflow;
    bool        noise;
    uint8_t     prev;
    int64_t     last_byte_us;
} rx_state_t;

static void uart_put_line(uart_port_t port, const char *s)
{
    size_t n = strlen(s);
    if (n > 0U) { (void)uart_write_bytes(port, s, n); }
    (void)uart_write_bytes(port, "\n", 1U);
}

static void rx_emit_line(rx_state_t *st)
{
    st->buf[st->len] = '\0';
    const bool overflow = st->overflow;
    const bool noise = st->noise;
    const size_t len = st->len;
    st->len = 0U; st->overflow = false; st->noise = false;
    if (overflow) { ESP_LOGW(TAG, "RX overflow on uart%d, line discarded", (int)st->port); return; }
    if (noise)    { ESP_LOGD(TAG, "uart%d noise discarded (%u bytes)", (int)st->port, (unsigned)len); return; }

    /* trim */
    char *p = st->buf;
    while ((*p == ' ') || (*p == '\t')) { p++; }
    size_t l = strlen(p);
    while ((l > 0U) && ((p[l - 1U] == ' ') || (p[l - 1U] == '\t'))) { p[--l] = '\0'; }
    if ((l == 0U) || (p[0] == '#')) { return; }       /* empty or log echo  */

    static line_msg_t lm;                              /* only this task      */
    lm.port = (uint8_t)st->port;
    (void)snprintf(lm.text, sizeof lm.text, "%s", p);
    if (xQueueSend(s_line_q, &lm, 0) != pdTRUE) {
        tx_simple((p[0] == '{') ? ROUTE_JSON : ROUTE_TEXT, true, "busy", "BUSY_QUEUE", "BUSY_QUEUE");
    }
}

static void rx_poll(rx_state_t *st, int64_t t)
{
    uint8_t in[64];
    int n = uart_read_bytes(st->port, in, sizeof in, 0);
    if (n < 0) { ESP_LOGW(TAG, "uart%d read error", (int)st->port); return; }
    for (int i = 0; i < n; i++) {
        const uint8_t b = in[i];
        st->last_byte_us = t;
        if ((b == (uint8_t)'\n') || (b == (uint8_t)'\r')) {
            /* CR+LF -> one echoed newline, one line */
            if (st->console && !((b == (uint8_t)'\n') && (st->prev == (uint8_t)'\r'))) {
                (void)uart_write_bytes(st->port, "\r\n", 2U);
            }
            if ((st->len > 0U) || st->overflow) { rx_emit_line(st); }
        } else if (st->console && ((b == 0x08U) || (b == 0x7FU))) {
            if (st->len > 0U) {                          /* backspace / DEL */
                st->len--;
                (void)uart_write_bytes(st->port, "\b \b", 3U);
            }
        } else if (st->len < (RX_LINE_MAX - 1U)) {
            st->buf[st->len++] = (char)b;
            if ((b < 0x20U) || (b > 0x7EU)) { st->noise = true; }   /* not text */
            if (st->console) { (void)uart_write_bytes(st->port, &b, 1U); }
        } else {
            st->overflow = true;                         /* keep consuming    */
        }
        st->prev = b;
    }
    /* idle line end on UART1 only (machine senders) */
    if (!st->console && ((st->len > 0U) || st->overflow) &&
        ((t - st->last_byte_us) >= ms_to_us(RX_IDLE_END_MS))) {
        rx_emit_line(st);
    }
}

static void uart_task(void *pv)
{
    (void)pv;
    static rx_state_t rx[2];
    static evt_msg_t m;
    static log_msg_t lg;
    rx[0].port = UART_FLASH_PORT; rx[0].console = true;
    rx[1].port = UART_SYS_PORT;   rx[1].console = false;
    task_wdt_subscribe();
    for (;;) {
        (void)esp_task_wdt_reset();
        const int64_t t = now_us();
        rx_poll(&rx[0], t);
        rx_poll(&rx[1], t);

        /* wait for an event (this is also the poll period), then drain */
        TickType_t wait = pdMS_TO_TICKS(UART_POLL_MS);
        while (xQueueReceive(s_evt_q, &m, wait) == pdTRUE) {
            wait = 0;
            switch ((route_t)m.route) {
            case ROUTE_TEXT:
                uart_put_line(UART_FLASH_PORT, m.text);
                uart_put_line(UART_SYS_PORT, m.text);
                break;
            case ROUTE_JSON:
                uart_put_line(UART_FLASH_PORT, m.json);
                uart_put_line(UART_SYS_PORT, m.json);
                break;
            case ROUTE_AUTO:
            default:
                uart_put_line(UART_FLASH_PORT, m.text);
                uart_put_line(UART_SYS_PORT, m.json);
                break;
            }
        }
        while (xQueueReceive(s_log_q, &lg, 0) == pdTRUE) {
            uart_put_line(UART_FLASH_PORT, lg.line);     /* logs: UART0 only  */
        }
    }
}

/* =============================================================================
 * STATUS task — heartbeat, progress, POSITION_UNKNOWN, health monitoring
 * ========================================================================== */
static bool snap_is_moving(const arm_snap_t *s)
{
    return (s->state == ST_STARTING) || (s->state == ST_DEPLOYING) || (s->state == ST_STOWING);
}

static void health_report(void)
{
    static const char *const names[5] = { "safety", "motion", "cmd", "uart", "status" };
    for (uint32_t i = 0U; i < 5U; i++) {
        if (s_task_h[i] == NULL) { continue; }
        const uint32_t free_b = (uint32_t)uxTaskGetStackHighWaterMark(s_task_h[i]);
        if (free_b < STACK_LOW_BYTES) {
            ESP_LOGW(TAG, "stack low: %s %lu B free", names[i], (unsigned long)free_b);
        } else {
            ESP_LOGD(TAG, "stack %s %lu B free", names[i], (unsigned long)free_b);
        }
    }
    ESP_LOGI(TAG, "health: evt_q %u/%u used, txdrop %lu",
             (unsigned)uxQueueMessagesWaiting(s_evt_q), (unsigned)EVT_QUEUE_LEN,
             (unsigned long)atomic_load(&s_tx_drop));
}

/* Per-arm state of the status-side checks for the current move. */
typedef struct {
    uint32_t move_seq;          /* move these checks belong to              */
    bool     active;
    bool     start_released;    /* start switch seen open                   */
    bool     fault_sent;
    int64_t  stall_win_us;      /* 0 = window not running                   */
    int32_t  stall_win_count;
    uint32_t drift_seq;         /* idle reference already warned for        */
    bool     lost_sent;
    bool     unk_warned;
    bool     both_sent;         /* idle BOTH_LIMITS requested               */
} mon_t;

static bool req_send(req_kind_t kind, arm_id_t id, fault_t f, uint32_t seq)
{
    const req_t r = { .kind = (uint8_t)kind, .arm = (uint8_t)id, .fault = (uint8_t)f, .seq = seq };
    if (xQueueSend(s_req_q, &r, 0) != pdTRUE) { ESP_LOGW(TAG, "req queue full - retry"); return false; }
    return true;
}

static int32_t snap_fwd_overtravel_limit(const arm_snap_t *s)
{
    const int32_t st = s->stroke_valid ? s->stroke : MAX_STROKE_COUNTS;
    int32_t m = st / 10;
    if (m < HALF_TURN_COUNTS) { m = HALF_TURN_COUNTS; }
    return st + m;
}

/* STALL / WRONG_DIR / OVERTRAVEL / SWITCH_STUCK / WRONG_LIMIT / BOTH_LIMITS
 * for a driving arm. Returns the fault to request, or FLT_NONE.
 * SHUB_BENCH_TEST: the encoder checks (STALL, WRONG_DIR, OVERTRAVEL,
 * SWITCH_STUCK) are off; switch checks stay on. */
static fault_t mon_check(mon_t *m, const arm_snap_t *s, int64_t t)
{
    const bool deploy    = (s->dir == DIR_DEPLOY);
    const bool deb_start = deploy ? s->ls1 : s->ls2;
    const int32_t travel = (s->count - s->start_count) * (deploy ? 1 : -1);

    if (s->ls1 && s->ls2) { return FLT_BOTH_LIMITS; }
    if (!m->start_released && !deb_start) { m->start_released = true; }
    if (m->start_released && s->start_was_pressed && deb_start) { return FLT_WRONG_LIMIT; }
    if (!s->start_was_pressed && deb_start)                     { return FLT_WRONG_LIMIT; }
    if (SHUB_BENCH_TEST) { return FLT_NONE; }

    if (!m->start_released && (travel > SWITCH_RELEASE_COUNTS)) { return FLT_SWITCH_STUCK; }

    /* stall: after the grace time, while driven and not holding on a limit */
    if ((s->leg_ms >= STALL_GRACE_MS) && !s->limit_hold && (s->duty > 0.0f)) {
        if (m->stall_win_us == 0) {
            m->stall_win_us = t;
            m->stall_win_count = s->count;
        } else if ((t - m->stall_win_us) >= ms_to_us(STALL_WINDOW_MS)) {
            int32_t moved = s->count - m->stall_win_count;
            if (moved < 0) { moved = -moved; }
            if (moved < STALL_MIN_COUNTS) { return FLT_STALL; }
            m->stall_win_us = t;
            m->stall_win_count = s->count;
        } else {
            /* window running */
        }
    } else {
        m->stall_win_us = 0;
    }

    if (travel < -WRONG_DIR_COUNTS) { return FLT_WRONG_DIR; }
    if (s->pos_valid) {
        if (deploy && (s->count > snap_fwd_overtravel_limit(s))) { return FLT_OVERTRAVEL; }
        if (!deploy && (s->count < -HALF_TURN_COUNTS))           { return FLT_OVERTRAVEL; }
    } else if (travel > MAX_STROKE_COUNTS) {
        return FLT_OVERTRAVEL;
    }
    return FLT_NONE;
}

static void status_task(void *pv)
{
    (void)pv;
    static sys_snap_t ss;
    static mon_t mon[ARM_COUNT];
    int64_t last_hb = now_us();
    int64_t last_prog = now_us();
    int64_t last_health = now_us();
    task_wdt_subscribe();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&last, pdMS_TO_TICKS(STATUS_PERIOD_MS));
        (void)esp_task_wdt_reset();
        (void)atomic_fetch_add(&s_status_beat, 1U);
        if (!snap_get(&ss)) { continue; }
        const int64_t t = now_us();

        const bool prog_due = (t - last_prog) >= ms_to_us(PROGRESS_PERIOD_MS);
        if (prog_due) { last_prog = t; }

        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            const arm_id_t id = (arm_id_t)i;
            const arm_snap_t *s = &ss.arm[i];
            mon_t *m = &mon[i];
            const bool driving = (s->state == ST_DEPLOYING) || (s->state == ST_STOWING);

            /* ---- fault checks while driving -> REQ_FAULT to motion ---- */
            if (driving) {
                if (!m->active || (m->move_seq != s->move_seq)) {
                    m->active = true;
                    m->move_seq = s->move_seq;
                    m->start_released = !s->start_was_pressed;
                    m->fault_sent = false;
                    m->stall_win_us = 0;
                }
                if (!m->fault_sent) {
                    const fault_t f = mon_check(m, s, t);
                    if (f != FLT_NONE) {
                        m->fault_sent = req_send(REQ_FAULT, id, f, s->move_seq);   /* retried if not queued */
                    }
                }
                if (prog_due) { ev_progress_snap(id, s); }
            } else {
                m->active = false;
            }

            if (snap_is_moving(s)) { m->lost_sent = false; m->both_sent = false; continue; }

            /* ---- idle: BOTH_LIMITS (switch fault at rest) ---- */
            if (s->ls1 && s->ls2 && (s->state != ST_FAULT)) {
                if (!m->both_sent) { m->both_sent = req_send(REQ_FAULT, id, FLT_BOTH_LIMITS, 0U); }
                continue;
            }
            m->both_sent = false;

            /* ---- idle: DRIFT (once per motor-off reference) ---- */
            if ((m->drift_seq != s->idle_seq) &&
                (labs((long)(s->count - s->idle_ref_count)) > (long)DRIFT_COUNTS)) {
                m->drift_seq = s->idle_seq;
                char dm[64];
                (void)snprintf(dm, sizeof dm, "DRIFT: %ld counts with motor off",
                               (long)(s->count - s->idle_ref_count));
                ev_arm_snap(ROUTE_AUTO, true, "warn", "none", id, s, false, dm);
                (void)req_send(REQ_SAVE, id, FLT_NONE, s->idle_seq);
            }

            /* ---- idle: POSITION_LOST (switch released at rest) ---- */
            const bool lost = ((s->state == ST_STOWED) && !s->ls1) || ((s->state == ST_DEPLOYED) && !s->ls2);
            if (lost && !m->lost_sent) {
                m->lost_sent = true;
                ev_arm_snap(ROUTE_AUTO, true, "position_lost", "none", id, s, false, "POSITION_LOST");
                (void)req_send(REQ_POSITION_LOST, id, FLT_NONE, s->idle_seq);
            } else if (!lost) {
                m->lost_sent = false;
            }

            /* ---- idle: POSITION_UNKNOWN (both open, no f1/f2, no position) ---- */
            const bool unknown = (s->state != ST_FAULT) && !s->ls1 && !s->ls2 &&
                                 (s->f1 == 0U) && (s->f2 == 0U) && !s->pos_valid;
            if (unknown && !m->unk_warned) {
                ev_arm_snap(ROUTE_AUTO, true, "warn", "none", id, s, false,
                            "POSITION_UNKNOWN: both switches open, no f1/f2, no position");
            }
            m->unk_warned = unknown;
        }

        if ((t - last_hb) >= ms_to_us(HEARTBEAT_PERIOD_MS)) {
            last_hb = t;
            status_report(ROUTE_AUTO, &ss);
        }
        if ((t - last_health) >= ms_to_us(STACK_REPORT_PERIOD_MS)) {
            last_health = t;
            health_report();
        }
    }
}

/* =============================================================================
 * Fatal paths
 * ========================================================================== */
static void halt_safe(const char *why)
{
    hal_force_pwm_pins_low_gpio();
    for (;;) {
        ESP_LOGE(TAG, "HALT SAFE: %s (motors off)", why);
        esp_rom_printf("HALT SAFE: %s\n", why);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* Stack overflow: PWM pins low, log, abort (-> reset, no auto-move). [§8.3/E] */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        (void)gpio_set_level(ARM_HW[i].pwm_gpio, 0U);
        esp_rom_gpio_connect_out_signal((uint32_t)ARM_HW[i].pwm_gpio, SIG_GPIO_OUT_IDX, false, false);
        esp_rom_gpio_pad_select_gpio((uint32_t)ARM_HW[i].pwm_gpio);
    }
    esp_rom_printf("STACK OVERFLOW in %s - motors off\n", (pcTaskName != NULL) ? pcTaskName : "?");
    abort();
}

/* =============================================================================
 * APP: boot
 * ========================================================================== */
static void boot_read_limits(void)
{
    for (uint32_t k = 0U; k < DEBOUNCE_SAMPLES; k++) {
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            arm_t *a = &s_arm[i];
            a->raw_deploy = hal_lim_raw(ARM_HW[i].lim_deploy_gpio);
            a->raw_stow   = hal_lim_raw(ARM_HW[i].lim_stow_gpio);
            debounce(a->raw_deploy, &a->lim_deploy, &a->deb_cnt_deploy);
            debounce(a->raw_stow,   &a->lim_stow,   &a->deb_cnt_stow);
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
    /* A switch stable for the whole window is accepted at its raw value. */
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        s_arm[i].lim_deploy = s_arm[i].raw_deploy;
        s_arm[i].lim_stow   = s_arm[i].raw_stow;
        s_arm[i].deb_cnt_deploy = 0U;
        s_arm[i].deb_cnt_stow   = 0U;
    }
}

static void boot_load_nvs(void)
{
    /* saved speed (same key/type as old firmware) */
    if (s_nvs_ok) {
        float v = 0.0f;
        size_t sz = sizeof v;
        esp_err_t se = nvs_get_blob(s_nvs, "saved_speed", &v, &sz);
        if ((se == ESP_OK) && (sz == sizeof v) && isfinite(v) && (fabsf(v) <= 1.0f)) {
            s_speed = v;
            ESP_LOGI(TAG, "Loaded speed %.2f", (double)v);
        } else {
            s_speed = DEFAULT_SPEED;
            ESP_LOGW(TAG, "No valid saved speed - default %.2f", (double)DEFAULT_SPEED);
        }
        s_speed_saved = s_speed;
    }
    /* move timeout */
    uint32_t to = 0U;
    if (s_nvs_ok && (nvs_get_u32(s_nvs, "timeout_s", &to) == ESP_OK) &&
        (to >= MOVE_TIMEOUT_MIN_S) && (to <= MOVE_TIMEOUT_MAX_S)) {
        atomic_store(&s_timeout_s, to);
    }
    ESP_LOGI(TAG, "Move timeout %lu s", (unsigned long)atomic_load(&s_timeout_s));

    /* per arm: one CRC-checked record; any bad value -> discard, defaults */
    static const char *const legacy_keys[] = { "cnt", "dcnt", "flags", "tms" };
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_t *a = &s_arm[i];
        const arm_id_t id = (arm_id_t)i;
        char k[16];
        arm_rec_t r;
        size_t sz = sizeof r;
        esp_err_t e = s_nvs_ok ? nvs_get_blob(s_nvs, nvs_key(id, "rec", k, sizeof k), &r, &sz) : ESP_ERR_INVALID_STATE;
        if ((e == ESP_OK) && (sz == sizeof r) && arm_rec_valid(&r)) {
            a->f1            = r.f1;
            a->f2            = r.f2;
            a->time_used_ms  = r.time_used_ms;
            a->count         = r.count;
            a->stroke        = r.stroke;
            a->stroke_valid  = (r.stroke_valid != 0U);
            a->rate_cps      = r.rate_cps;
            a->stop_time_ms  = r.stop_time_ms;
            a->dist_restored = true;
            a->pos_valid     = (r.pos_valid != 0U);
            s_rec_saved[i]    = r;
            s_rec_saved_ok[i] = true;
        } else {
            if (e != ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG, "%s NVS record discarded (%s) - defaults", ARM_HW[i].name,
                         (e == ESP_OK) ? "CRC/range" : esp_err_to_name(e));
            }
            a->f1 = 0U; a->f2 = 0U; a->time_used_ms = 0U; a->count = 0;
            a->stroke = 0; a->stroke_valid = false; a->rate_cps = 0U; a->stop_time_ms = 0U;
            a->dist_restored = false;
            a->pos_valid     = false;
            s_rec_saved_ok[i] = false;
        }
        /* v1 per-value keys are replaced by the record */
        for (uint32_t kk = 0U; s_nvs_ok && (kk < (sizeof legacy_keys / sizeof legacy_keys[0])); kk++) {
            if (nvs_erase_key(s_nvs, nvs_key(id, legacy_keys[kk], k, sizeof k)) == ESP_OK) {
                (void)nvs_commit(s_nvs);
            }
        }
        a->route = ROUTE_AUTO;
        a->last_dir = DIR_NONE;
        a->move_cmd = "none";
        ESP_LOGI(TAG, "%s saved: pos=%ld f1=%u f2=%u time=%lu ms stroke=%ld%s rate=%lu cps", ARM_HW[i].name,
                 (long)a->count, (unsigned)a->f1, (unsigned)a->f2, (unsigned long)a->time_used_ms,
                 (long)a->stroke, a->stroke_valid ? "" : "(none)", (unsigned long)a->rate_cps);
    }
}

static TaskHandle_t create_task(TaskFunction_t fn, const char *name, uint32_t stack_bytes,
                                UBaseType_t prio, StackType_t *stack, StaticTask_t *tcb)
{
    /* TWDT subscription is done by the task itself (task_wdt_subscribe). */
    TaskHandle_t h = xTaskCreateStatic(fn, name, stack_bytes, NULL, prio, stack, tcb);
    if (h == NULL) { halt_safe(name); }
    return h;
}

void app_main(void)
{
    /* 1. motors off before anything else */
    hal_force_pwm_pins_low_gpio();
    hal_dir_pins_init();

    /* 2. servo output configured, pulse unchanged (lock not moved) */
    esp_err_t e = hal_servo_init();

    /* 3. identity + reset cause */
    const esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGI(TAG, "Sensor Hub Controller v%s (%s) reset=%s", FW_VERSION, FW_BUILD_DATE, reset_reason_str(rr));

    /* 4. NVS + saved data */
    esp_err_t ne = nvs_flash_init();
    if ((ne == ESP_ERR_NVS_NO_FREE_PAGES) || (ne == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_LOGW(TAG, "NVS erase + re-init");
        ne = nvs_flash_erase();
        if (ne == ESP_OK) { ne = nvs_flash_init(); }
    }
    if (ne == ESP_OK) { ne = nvs_open("storage", NVS_READWRITE, &s_nvs); }
    s_nvs_ok = (ne == ESP_OK);
    if (!s_nvs_ok) { ESP_LOGE(TAG, "NVS unavailable (%s) - running without persistence", esp_err_to_name(ne)); }
    boot_load_nvs();

    /* 5. mutexes + queues + log hook (console sharing) */
    s_motor_mtx = xSemaphoreCreateMutexStatic(&s_motor_mtx_buf);
    s_snap_mtx  = xSemaphoreCreateMutexStatic(&s_snap_mtx_buf);
    s_line_q   = xQueueCreateStatic(LINE_QUEUE_LEN, sizeof(line_msg_t), s_line_q_store, &s_line_q_buf);
    s_cmd_q    = xQueueCreateStatic(CMD_QUEUE_LEN, sizeof(cmd_t), s_cmd_q_store, &s_cmd_q_buf);
    s_req_q    = xQueueCreateStatic(REQ_QUEUE_LEN, sizeof(req_t), s_req_q_store, &s_req_q_buf);
    s_evt_q    = xQueueCreateStatic(EVT_QUEUE_LEN, sizeof(evt_msg_t), s_evt_q_store, &s_evt_q_buf);
    s_log_q    = xQueueCreateStatic(LOG_QUEUE_LEN, sizeof(log_msg_t), s_log_q_store, &s_log_q_buf);
    if ((s_motor_mtx == NULL) || (s_snap_mtx == NULL) || (s_line_q == NULL) || (s_cmd_q == NULL) ||
        (s_req_q == NULL) || (s_evt_q == NULL) || (s_log_q == NULL)) {
        halt_safe("queue/mutex create");
    }
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf_hook);

    /* 6. peripherals */
    if (e == ESP_OK) { e = hal_uart_init(UART_FLASH_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); }
    if (e == ESP_OK) { e = hal_uart_init(UART_SYS_PORT, UART_SYS_TX_GPIO, UART_SYS_RX_GPIO); }
    for (uint32_t i = 0U; (i < (uint32_t)ARM_COUNT) && (e == ESP_OK); i++) { e = hal_motor_init((arm_id_t)i); }
    hal_usb_pads_release();          /* GPIO19/20 are encoder inputs */
    for (uint32_t i = 0U; (i < (uint32_t)ARM_COUNT) && (e == ESP_OK); i++) { e = hal_encoder_init((arm_id_t)i); }
    if (e == ESP_OK) { e = hal_limits_init(); }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(e));
        halt_safe("peripheral init");
    }

    /* 7. initial arm states from switches + NVS record (boot_classify) */
    boot_read_limits();
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        arm_id_t id = (arm_id_t)i;
        (void)hal_enc_read(id);          /* fresh PCNT after init is 0 */
        s_arm[i].pcnt_prev = 0;
        boot_classify(id, s_rec_saved_ok[i], s_rec_saved_ok[i] && (s_rec_saved[i].pos_valid != 0U));
        s_arm[i].idle_ref_count = s_arm[i].count;
        s_arm[i].idle_seq = 1U;          /* #5: status DRIFT check live from boot */
        nvs_save_arm(id);                /* switch truth clears stale f1/f2 */
    }

    /* 8. each lock to match its arm (set only at deploy); both servos settle
     *    together (blocking settle is fine at boot) */
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        const bool ok = hal_servo_write_us((arm_id_t)i, (s_arm[i].state == ST_DEPLOYED) ? SERVO_LOCK_US : SERVO_UNLOCK_US);
        s_arm[i].servo_err = !ok;
    }
    vTaskDelay(pdMS_TO_TICKS(LOCK_SETTLE_MS));
    for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
        /* failed write -> UNKNOWN (never UNLOCKED): motor cannot run */
        s_lock_state[i] = s_arm[i].servo_err ? LOCK_UNKNOWN
                        : ((s_arm[i].state == ST_DEPLOYED) ? LOCK_LOCKED : LOCK_UNLOCKED);
    }
    snap_publish(now_us());

    /* 9. TWDT (panic on timeout) */
    const esp_task_wdt_config_t wcfg = {
        .timeout_ms     = TWDT_TIMEOUT_MS,
        .idle_core_mask = (1U << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1U,
        .trigger_panic  = true,
    };
#if CONFIG_ESP_TASK_WDT_INIT
    esp_err_t we = esp_task_wdt_reconfigure(&wcfg);   /* IDF already started it */
#else
    esp_err_t we = esp_task_wdt_init(&wcfg);
#endif
    if (we != ESP_OK) { halt_safe("TWDT config"); }

    /* 10. BOOT message (+ warnings) — queued before the tasks start */
    if (SHUB_BENCH_TEST) {
        tx_simple(ROUTE_AUTO, true, "warn", "BENCH_TEST", "WARN BENCH_TEST build: encoder fault checks OFF - not for use");
    }
    if (reset_reason_abnormal(rr)) {
        char w[48];
        (void)snprintf(w, sizeof w, "WARN reset=%s", reset_reason_str(rr));
        tx_simple(ROUTE_AUTO, true, "warn", "ABNORMAL_RESET", w);
    }
    {
        char t[TX_TEXT_MAX];
        char j[TX_JSON_MAX];
        (void)snprintf(t, sizeof t, "BOOT v%s %s reset=%s speed=%.2f timeout=%lus",
                       FW_VERSION, FW_BUILD_DATE, reset_reason_str(rr), (double)fabsf(s_speed),
                       (unsigned long)atomic_load(&s_timeout_s));
        (void)snprintf(j, sizeof j,
                       "{\"evt\":\"boot\",\"ver\":\"%s\",\"build\":\"%s\",\"reset\":\"%s\",\"speed\":%.2f,"
                       "\"timeout_s\":%lu}",
                       FW_VERSION, FW_BUILD_DATE, reset_reason_str(rr), (double)fabsf(s_speed),
                       (unsigned long)atomic_load(&s_timeout_s));
        tx_post(ROUTE_AUTO, true, t, j);
        for (uint32_t i = 0U; i < (uint32_t)ARM_COUNT; i++) {
            ev_arm(ROUTE_AUTO, true, "boot", "none", (arm_id_t)i, s_arm[i].state != ST_FAULT, "BOOT");
        }
    }
    s_last_nvs_save_us = now_us();

    /* 11. tasks — output/consumers first, motion last so nothing drives
     *     before every supervisor exists */
    s_task_h[3] = create_task(uart_task,   "uart_task",   UART_STACK_BYTES,   UART_PRIO,   s_uart_stack,   &s_uart_tcb);
    s_task_h[4] = create_task(status_task, "status_task", STATUS_STACK_BYTES, STATUS_PRIO, s_status_stack, &s_status_tcb);
    s_task_h[2] = create_task(cmd_task,    "cmd_task",    CMD_STACK_BYTES,    CMD_PRIO,    s_cmd_stack,    &s_cmd_tcb);
    s_task_h[0] = create_task(safety_task, "safety_task", SAFETY_STACK_BYTES, SAFETY_PRIO, s_safety_stack, &s_safety_tcb);
    s_task_h[1] = create_task(motion_task, "motion_task", MOTION_STACK_BYTES, MOTION_PRIO, s_motion_stack, &s_motion_tcb);
    s_motion_task = s_task_h[1];              /* limit ISR target */
    /* app_main returns; motors stay off until a command arrives. */
}
