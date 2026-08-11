/**
 * @file keypad.c
 * @brief Keypad component - public API + INT-driven event engine.
 *
 * The engine mirrors the OEM behavior (wire-proven):
 *  - IDLE: low-power chip config, task blocked on the INT falling edge.
 *  - first touch -> ACTIVE: OEM active config + poll every poll_period_ms.
 *  - idle_timeout_ms without touch -> full idle burst -> IDLE.
 *
 * Key acceptance (decision D12, evidence-based): an episode runs while the
 * touch mask is non-zero (tolerating zero_tolerance zero polls). The real
 * key is the bit common to every sample (the ghost always arrives later);
 * if the intersection keeps >1 bit, column-ghost disambiguation drops the
 * bit whose neighbor ABOVE is also present (ghosts grow downward, measured
 * 6/6). Still ambiguous episodes are rejected and counted.
 */
#include "keypad.h"
#include "keypad_gtx312l.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "keypad";

static i2c_master_bus_handle_t s_bus;
static gtx312l_tune_t s_tune;
static gpio_num_t s_int_pin = GPIO_NUM_NC;
static keypad_config_t s_cfg;

/* ------------------------- event engine state ------------------------ */

static SemaphoreHandle_t s_int_sem;
static TaskHandle_t s_task_handle;
static QueueHandle_t s_event_queue;
static volatile bool s_stop;
static volatile bool s_masked;
static volatile keypad_state_t s_state = KEYPAD_STATE_IDLE;

typedef struct {
    bool     active;
    bool     accepted;
    bool     doomed;        /* intersection lost or ambiguous: no accept */
    uint16_t intersection;  /* bits common to every sample of the episode */
    uint8_t  count;         /* in-episode polls */
    uint8_t  zero_count;    /* consecutive zero polls (release debounce) */
    keypad_key_t key;
} episode_t;

static episode_t s_ep;
static int64_t s_last_touch_ms;
static keypad_stats_t s_stats;

/* Touch bit (channel-1) -> key. Wire-proven 2026-08-10: K_n = channel n. */
static const keypad_key_t s_key_from_bit[12] = {
    KEYPAD_KEY_6,    /* K1  */  KEYPAD_KEY_3,    /* K2  */
    KEYPAD_KEY_9,    /* K3  */  KEYPAD_KEY_5,    /* K4  */
    KEYPAD_KEY_HASH, /* K5  */  KEYPAD_KEY_STAR, /* K6  */
    KEYPAD_KEY_0,    /* K7  */  KEYPAD_KEY_7,    /* K8  */
    KEYPAD_KEY_8,    /* K9  */  KEYPAD_KEY_4,    /* K10 */
    KEYPAD_KEY_2,    /* K11 */  KEYPAD_KEY_1,    /* K12 */
};

static const char *const s_key_label[] = {
    [KEYPAD_KEY_6] = "6", [KEYPAD_KEY_3] = "3", [KEYPAD_KEY_9] = "9",
    [KEYPAD_KEY_5] = "5", [KEYPAD_KEY_HASH] = "#", [KEYPAD_KEY_STAR] = "*",
    [KEYPAD_KEY_0] = "0", [KEYPAD_KEY_7] = "7", [KEYPAD_KEY_8] = "8",
    [KEYPAD_KEY_4] = "4", [KEYPAD_KEY_2] = "2", [KEYPAD_KEY_1] = "1",
};

/* Column-neighbor ABOVE each channel (-1 = top row). Columns measured on
 * the wire: K12>K10>K8>K6, K11>K4>K9>K7, K2>K1>K3>K5 (ghosts grow down). */
static const int8_t s_above_of[12] = {
    /* K1 */  1,  /* K2 */ -1,  /* K3 */  0,  /* K4 */ 10,
    /* K5 */  2,  /* K6 */  7,  /* K7 */  8,  /* K8 */  9,
    /* K9 */  3,  /* K10*/ 11,  /* K11*/ -1,  /* K12*/ -1,
};

/** @brief Drop bits whose column-neighbor-above is also set (ghost disambig). */
static uint16_t disambiguate(uint16_t bits)
{
    for (uint8_t b = 0; b < 12; b++) {
        int8_t above = s_above_of[b];
        if ((bits & (1U << b)) && above >= 0 && (bits & (1U << above))) {
            bits &= ~(1U << b);
        }
    }
    return bits;
}

static void emit_event(keypad_key_t key, bool pressed)
{
    if (s_event_queue == NULL) {
        return;
    }
    keypad_event_t ev = {
        .key = key,
        .pressed = pressed,
        .timestamp_ms = esp_timer_get_time() / 1000,
    };
    if (xQueueSend(s_event_queue, &ev, 0) != pdTRUE) {
        ESP_LOGW(TAG, "event queue full, dropping %s '%s'",
                 pressed ? "PRESSED" : "RELEASED", keypad_key_label(key));
    }
}

/** @brief Feed one ACTIVE-mode poll sample into the episode machine. */
static void engine_poll(uint16_t mask)
{
    int64_t now = esp_timer_get_time() / 1000;

    if (s_masked) {
        /* RF window: user is present (stay awake) but touch is noise. */
        s_last_touch_ms = now;
        if (s_ep.active && s_ep.accepted) {
            emit_event(s_ep.key, false);
        }
        s_ep = (episode_t) { 0 };
        return;
    }

    if (mask != 0) {
        s_last_touch_ms = now;
        s_ep.zero_count = 0;

        if (!s_ep.active) {
            s_ep = (episode_t) { .active = true, .intersection = mask, .count = 1 };
        } else {
            s_ep.intersection &= mask;
            s_ep.count++;
        }

        if (s_ep.intersection == 0) {
            s_ep.doomed = true;     /* no common bit left: can never recover */
        }

        if (s_ep.active && !s_ep.accepted && !s_ep.doomed &&
            s_ep.count >= s_cfg.accept_min_polls) {
            uint16_t bits = disambiguate(s_ep.intersection);

            if (bits != 0 && (bits & (bits - 1)) == 0) {
                uint8_t bit = (uint8_t)__builtin_ctz(bits);
                s_ep.key = keypad_key_from_bit(bit);
                s_ep.accepted = true;
                s_stats.accepted++;
                emit_event(s_ep.key, true);
            } else {
                /* counted at episode end (rejected_ambiguous) */
                s_ep.doomed = true;
                ESP_LOGW(TAG, "episode rejected (mask history intersection "
                              "0x%03X ambiguous)", s_ep.intersection);
            }
        }
        return;
    }

    /* mask == 0: release debounce, then episode end. */
    if (s_ep.active && ++s_ep.zero_count >= s_cfg.zero_tolerance) {
        if (s_ep.accepted) {
            emit_event(s_ep.key, false);
        } else if (s_ep.count >= s_cfg.accept_min_polls) {
            s_stats.rejected_ambiguous++;
        } else {
            s_stats.rejected_short++;
        }
        s_ep = (episode_t) { 0 };
    }

    /* OEM parity: ~6 s without touch -> back to the low-power config. */
    if (now - s_last_touch_ms >= (int64_t)s_cfg.idle_timeout_ms) {
        if (gtx312l_apply_preset(GTX312L_PRESET_IDLE, &s_tune) == ESP_OK) {
            s_state = KEYPAD_STATE_IDLE;
            ESP_LOGI(TAG, "idle timeout -> low-power config");
        } else {
            ESP_LOGW(TAG, "idle config write failed, staying active");
            s_last_touch_ms = now;      /* retry after another timeout */
        }
    }
}

static void engine_wake(void)
{
    uint16_t mask = 0;

    /* OEM order: read TOUCH_OUT first, then write the active config. */
    if (keypad_read_touch_mask(&mask) != ESP_OK) {
        ESP_LOGW(TAG, "wake read failed");
    }
    if (gtx312l_apply_preset(GTX312L_PRESET_ACTIVE, &s_tune) != ESP_OK) {
        ESP_LOGW(TAG, "active config write failed");
    }

    s_last_touch_ms = esp_timer_get_time() / 1000;
    s_state = KEYPAD_STATE_ACTIVE;

    if (mask != 0) {
        engine_poll(mask);      /* the waking touch already started an episode */
    }
    ESP_LOGD(TAG, "wake (mask=0x%03X)", mask);
}

static void IRAM_ATTR int_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_int_sem, &higher_prio_woken);
    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

static void keypad_task(void *arg)
{
    while (!s_stop) {
        if (s_state == KEYPAD_STATE_IDLE) {
            xSemaphoreTake(s_int_sem, 0);   /* drain stale edges */

            /* INT is level-compatible: if the line is already low (touch in
             * progress while arming), wake without waiting for an edge. */
            if (gpio_get_level(s_int_pin) == 0 ||
                xSemaphoreTake(s_int_sem, pdMS_TO_TICKS(200)) == pdTRUE) {
                engine_wake();
            }
            continue;
        }

        uint16_t mask = 0;
        if (keypad_read_touch_mask(&mask) == ESP_OK) {
            engine_poll(mask);
        } else {
            ESP_LOGW(TAG, "poll read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(s_cfg.poll_period_ms));
    }

    s_task_handle = NULL;
    vTaskDelete(NULL);
}

/* --------------------------- public API ------------------------------ */

esp_err_t keypad_init(const keypad_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = cfg->i2c_port,
        .sda_io_num        = cfg->sda_pin,
        .scl_io_num        = cfg->scl_pin,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* The lock board relies on the GTX312L internal pull-ups (weak):
         * keep the S3 ones enabled as a safety net. External 4k7-10k are
         * still recommended on the bench if the clock is raised. */
        .flags = { .enable_internal_pullup = true },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c bus init failed");

    ESP_RETURN_ON_ERROR(gtx312l_init(s_bus, cfg->i2c_freq_hz), TAG, "gtx312l attach failed");

    gpio_config_t int_cfg = {
        .pin_bit_mask = 1ULL << cfg->int_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   /* inert when unwired */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,    /* the ISR arms in keypad_start */
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "int pin config failed");

    s_int_pin = cfg->int_pin;
    s_cfg = *cfg;
    s_tune = (gtx312l_tune_t) {
        .sens_active = cfg->sens_active,
        .sens_idle   = cfg->sens_idle,
        .idle_time   = 0x03,   /* OEM idle sensing: 330 ms */
        .idle_suffix = 0x03,
    };

    ESP_LOGI(TAG, "init OK (i2c %lu Hz, INT=GPIO%d)", (unsigned long)cfg->i2c_freq_hz,
             (int)cfg->int_pin);
    return ESP_OK;
}

esp_err_t keypad_self_test(void)
{
    return gtx312l_self_test();
}

esp_err_t keypad_boot_config(void)
{
    return gtx312l_boot_sequence(&s_tune);
}

esp_err_t keypad_apply_active_config(void)
{
    return gtx312l_apply_preset(GTX312L_PRESET_ACTIVE, &s_tune);
}

esp_err_t keypad_apply_idle_config(void)
{
    return gtx312l_apply_preset(GTX312L_PRESET_IDLE, &s_tune);
}

esp_err_t keypad_read_reg(uint8_t reg, uint8_t *val)
{
    return gtx312l_read_reg(reg, val);
}

esp_err_t keypad_write_reg(uint8_t reg, uint8_t val)
{
    return gtx312l_write_reg(reg, val);
}

esp_err_t keypad_read_touch_mask(uint16_t *mask12)
{
    return gtx312l_read_touch_out(mask12);
}

int keypad_get_int_level(void)
{
    /* INT is open-drain active-low: 0 = asserted. */
    return (s_int_pin == GPIO_NUM_NC) ? 1 : gpio_get_level(s_int_pin);
}

esp_err_t keypad_scan_bus(void)
{
    ESP_RETURN_ON_FALSE(s_bus != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    printf("scanning 7-bit address space 0x08..0x77:\n");
    int found = 0;
    for (uint16_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
            printf("  ACK at 0x%02X (wire bytes W=0x%02X R=0x%02X)\n",
                   addr, addr << 1, (addr << 1) | 1);
            found++;
        }
    }
    if (found == 0) {
        printf("  (no ACKs - frame is not reaching a live chip)\n");
    }
    return ESP_OK;
}

esp_err_t keypad_start(QueueHandle_t event_queue)
{
    ESP_RETURN_ON_FALSE(s_task_handle == NULL, ESP_ERR_INVALID_STATE, TAG,
                        "engine already running");

    /* Known starting point: OEM low-power config, waiting for INT. */
    ESP_RETURN_ON_ERROR(gtx312l_apply_preset(GTX312L_PRESET_IDLE, &s_tune),
                        TAG, "idle config write failed");

    s_event_queue = event_queue;
    s_ep = (episode_t) { 0 };
    s_stats = (keypad_stats_t) { 0 };
    s_masked = false;
    s_stop = false;
    s_state = KEYPAD_STATE_IDLE;
    s_last_touch_ms = esp_timer_get_time() / 1000;

    if (s_int_sem == NULL) {
        s_int_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_int_sem != NULL, ESP_ERR_NO_MEM, TAG, "sem create");
    }
    xSemaphoreTake(s_int_sem, 0);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   /* already installed */
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(s_int_pin, GPIO_INTR_NEGEDGE),
                        TAG, "intr type");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_int_pin, int_isr_handler, NULL),
                        TAG, "isr add");

    BaseType_t ok = xTaskCreate(keypad_task, "keypad", 3072, NULL, 5, &s_task_handle);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "task create");

    ESP_LOGI(TAG, "engine started (idle config armed, INT on GPIO%d)",
             (int)s_int_pin);
    return ESP_OK;
}

esp_err_t keypad_stop(void)
{
    ESP_RETURN_ON_FALSE(s_task_handle != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "engine not running");

    s_stop = true;
    xSemaphoreGive(s_int_sem);      /* unblock the IDLE wait */

    for (int i = 0; i < 200 && s_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    gpio_isr_handler_remove(s_int_pin);
    return (s_task_handle == NULL) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t keypad_mask_touch(bool mask)
{
    s_masked = mask;
    ESP_LOGI(TAG, "touch %s", mask ? "MASKED (RF window)" : "unmasked");
    return ESP_OK;
}

keypad_state_t keypad_get_state(void)
{
    return s_state;
}

esp_err_t keypad_get_stats(keypad_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(stats != NULL, ESP_ERR_INVALID_ARG, TAG, "stats NULL");
    *stats = s_stats;
    return ESP_OK;
}

esp_err_t keypad_prepare_sleep(void)
{
    ESP_RETURN_ON_ERROR(gtx312l_apply_preset(GTX312L_PRESET_IDLE, &s_tune),
                        TAG, "idle config write failed");
    /* INT is open-drain active-low. Deep-sleep GPIO wake on the S3 uses
     * ext1 (RTC GPIOs 0-21); the ANY_LOW mode exists on S2/S3/C6/H2.
     * The pull-up set in keypad_init() is held during sleep (HOLD). */
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup_io(1ULL << s_int_pin,
                                                        ESP_EXT1_WAKEUP_ANY_LOW),
                        TAG, "gpio wake");
    return ESP_OK;
}

keypad_key_t keypad_key_from_bit(uint8_t bit)
{
    return (bit < 12) ? s_key_from_bit[bit] : KEYPAD_KEY_NONE;
}

const char *keypad_key_label(keypad_key_t key)
{
    if (key >= KEYPAD_KEY_6 && key <= KEYPAD_KEY_1) {
        return s_key_label[key];
    }
    return "?";
}
