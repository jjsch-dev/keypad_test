/**
 * @file keypad.h
 * @brief Capacitive keypad component (GTX312L, 12 keys) - public API.
 *
 * Step 1: I2C bring-up, self-test, raw register access, OEM config presets.
 * Step 2: INT-driven event engine (state machine + debounce D12 + key queue).
 *
 * Behavioral reference: OEM FR8018H firmware, wire-proven with the BP5
 * sniffer (see PROJECT_NOTES.md "Stage 3"). Parity by default (D2);
 * parameters come from Kconfig and can be overridden at runtime through
 * keypad_config_t (D11 hybrid).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Key identifiers (touch-channel order matches the PCB silkscreen K_n). */
typedef enum {
    KEYPAD_KEY_NONE = 0,
    KEYPAD_KEY_6,        /* K1  (bit 0)  */
    KEYPAD_KEY_3,        /* K2  (bit 1)  */
    KEYPAD_KEY_9,        /* K3  (bit 2)  */
    KEYPAD_KEY_5,        /* K4  (bit 3)  */
    KEYPAD_KEY_HASH,     /* K5  (bit 4)  */
    KEYPAD_KEY_STAR,     /* K6  (bit 5)  */
    KEYPAD_KEY_0,        /* K7  (bit 6)  */
    KEYPAD_KEY_7,        /* K8  (bit 7)  */
    KEYPAD_KEY_8,        /* K9  (bit 8)  */
    KEYPAD_KEY_4,        /* K10 (bit 9)  */
    KEYPAD_KEY_2,        /* K11 (bit 10) */
    KEYPAD_KEY_1,        /* K12 (bit 11) */
} keypad_key_t;

typedef struct {
    i2c_port_t  i2c_port;       /* bus to create/own (the keypad is its only user for now) */
    gpio_num_t  sda_pin;
    gpio_num_t  scl_pin;
    gpio_num_t  int_pin;        /* open-drain active-low INT; RTC-capable GPIO (deep-sleep wake, D9) */
    uint32_t    i2c_freq_hz;    /* OEM parity: 37 kHz (weak pull-ups on the lock board) */
    uint8_t     sens_active;    /* GTX312L sensitivity while tracking (lower = more sensitive) */
    uint8_t     sens_idle;      /* GTX312L sensitivity while idle */
    uint8_t     accept_min_polls;   /* consecutive in-episode polls to accept a key (~80 ms) */
    uint8_t     zero_tolerance;     /* consecutive zero polls that end an episode (debounce) */
    uint32_t    poll_period_ms;     /* active polling period (OEM: 20 ms) */
    uint32_t    idle_timeout_ms;    /* no-touch time before going back to idle (~6 s) */
} keypad_config_t;

/** @brief Compile-time defaults from Kconfig (OEM parity). */
#define KEYPAD_CONFIG_DEFAULT() {                 \
    .i2c_port        = I2C_NUM_0,                 \
    .sda_pin         = CONFIG_KEYPAD_SDA_GPIO,    \
    .scl_pin         = CONFIG_KEYPAD_SCL_GPIO,    \
    .int_pin         = CONFIG_KEYPAD_INT_GPIO,    \
    .i2c_freq_hz     = CONFIG_KEYPAD_I2C_FREQ_HZ, \
    .sens_active     = CONFIG_KEYPAD_SENS_ACTIVE, \
    .sens_idle       = CONFIG_KEYPAD_SENS_IDLE,   \
    .accept_min_polls = CONFIG_KEYPAD_ACCEPT_MIN_POLLS, \
    .zero_tolerance   = CONFIG_KEYPAD_ZERO_TOLERANCE_POLLS, \
    .poll_period_ms  = CONFIG_KEYPAD_POLL_PERIOD_MS,    \
    .idle_timeout_ms = CONFIG_KEYPAD_IDLE_TIMEOUT_MS,   \
}

/** @brief Key event delivered through the application queue. */
typedef struct {
    keypad_key_t key;
    bool         pressed;       /* true = PRESSED (accepted), false = RELEASED */
    int64_t      timestamp_ms;  /* esp_timer-based */
} keypad_event_t;

typedef enum {
    KEYPAD_STATE_IDLE = 0,      /* low-power config armed, waiting for INT */
    KEYPAD_STATE_ACTIVE,        /* tracking touches, polling at poll_period_ms */
} keypad_state_t;

/**
 * @brief Initialize the I2C bus, the INT line and attach the GTX312L.
 *
 * Does NOT write any chip register: run keypad_self_test() /
 * keypad_boot_config() next. INT stays an input with pull-up, so the line
 * is inert if unwired.
 */
esp_err_t keypad_init(const keypad_config_t *cfg);

/**
 * @brief Chip identity + fresh-POR self-test (mirrors the OEM boot checks).
 *
 * CHIP_ID (01h) must read 0xB2 (CTRL=OPEN) and SENSITIVITY1 (20h) should
 * still hold its reset value 0x0F. The first check is fatal, the second is
 * a warning (the chip keeps registers across MCU resets).
 */
esp_err_t keypad_self_test(void);

/**
 * @brief Full OEM power-on sequence: magic 0Bh=0x5B, then ACTIVE pass,
 *        then IDLE pass (as captured at the lock's boot).
 */
esp_err_t keypad_boot_config(void);

/** @brief Write one OEM config preset (wire-proven bursts). */
esp_err_t keypad_apply_active_config(void);
esp_err_t keypad_apply_idle_config(void);

esp_err_t keypad_read_reg(uint8_t reg, uint8_t *val);
esp_err_t keypad_write_reg(uint8_t reg, uint8_t val);

/** @brief Read the 12-key touch mask (bit n-1 = channel K_n). */
esp_err_t keypad_read_touch_mask(uint16_t *mask12);

/** @brief Current level of the INT line (0 = asserted, open-drain active-low). */
int keypad_get_int_level(void);

/**
 * @brief Bench diagnostic: probe the whole 7-bit address space and print
 *        every address that ACKs. If the GTX312L moved (CTRL strap state),
 *        this finds where it went.
 */
esp_err_t keypad_scan_bus(void);

/* ---------------------------------------------------------------------
 * Step 2: event state machine (INT wake + OEM-mirror modes + D12 debounce)
 * ------------------------------------------------------------------- */

/** @brief Acceptance statistics (bench tuning aid). */
typedef struct {
    uint32_t accepted;           /* episodes accepted (PRESSED emitted) */
    uint32_t rejected_short;     /* episodes shorter than accept_min_polls */
    uint32_t rejected_ambiguous; /* multi-bit even after column disambiguation */
} keypad_stats_t;

/**
 * @brief Start the event engine.
 *
 * Applies the OEM idle preset (pulse INT, low-power sensing), arms the INT
 * falling-edge interrupt and spawns the keypad task:
 *
 *   IDLE   - no polling; the first touch asserts INT and wakes the task.
 *   ACTIVE - OEM active config + polling every poll_period_ms with the D12
 *            debounce: an episode (touch mask != 0) of >= accept_min_polls
 *            whose samples share exactly one bit (after dropping
 *            column-neighbor-below ghost bits) emits PRESSED right away
 *            (OEM beep timing); the episode end emits RELEASED. After
 *            idle_timeout_ms without touch the idle preset is rewritten
 *            and the task goes back to IDLE.
 *
 * Accepted key events are pushed to the application-provided queue
 * (ownership stays with the caller; recommended depth >= 8).
 *
 * @param event_queue  Queue (of keypad_event_t) created by the caller.
 */
esp_err_t keypad_start(QueueHandle_t event_queue);

/** @brief Stop the event engine (task exits after the current poll). */
esp_err_t keypad_stop(void);

/**
 * @brief Mute/unmute touch events while the RF reader polls cards.
 *
 * 13.56 MHz bursts inject touch noise (wire-proven); single-bit stable
 * noise would pass the debounce, so masking during RF is mandatory.
 * Masking aborts the current episode (a RELEASED is emitted if a key was
 * accepted) and keeps the engine awake.
 */
esp_err_t keypad_mask_touch(bool mask);

/** @brief Current engine state (IDLE = low-power config armed). */
keypad_state_t keypad_get_state(void);

/** @brief Copy the acceptance statistics (bench tuning aid). */
esp_err_t keypad_get_stats(keypad_stats_t *stats);

/**
 * @brief Prepare deep-sleep: rewrite the idle config and arm the INT pin
 *        as an ext1 wake source (ESP_EXT1_WAKEUP_ANY_LOW). INT must be an
 *        RTC-capable GPIO (0-21 on the S3). The application owns the
 *        actual esp_deep_sleep_start() call.
 */
esp_err_t keypad_prepare_sleep(void);

/** @brief Map a touch bit (0..11) to its keypad key; KEYPAD_KEY_NONE if invalid. */
keypad_key_t keypad_key_from_bit(uint8_t bit);

/** @brief Printable label for a key ("1".."0", "*", "#"). */
const char *keypad_key_label(keypad_key_t key);

#ifdef __cplusplus
}
#endif
