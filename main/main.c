/**
 * @file main.c
 * @brief keypad_test - interactive console harness for the keypad component
 *        (step 1: I2C bring-up, self-test, raw touch monitor;
 *         step 2: INT-driven event engine with D12 debounce).
 *
 * Commands:
 *   selftest            CHIP_ID + SENSITIVITY1 checks (OEM boot parity)
 *   scan                probe all 7-bit I2C addresses
 *   bootcfg             full OEM power-on sequence (magic + active + idle)
 *   config idle|active  write one OEM config preset
 *   dump                hex dump of registers 01h..2Dh
 *   read <reg>          read one register (hex)
 *   write <reg> <val>   write one register (hex)
 *   monitor [n]         raw TOUCH_OUT + INT view, n polls (default 500 = 10 s)
 *   events [seconds]    run the debounced event engine (default 30 s) + stats
 *   mask on|off         mute/unmute touch (RF card-reader window)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "keypad.h"

static const char *TAG = "keypad_test";

/* ------------------------------------------------------------------ */
static int cmd_selftest(int argc, char **argv)
{
    esp_err_t err = keypad_self_test();
    printf("selftest: %s\n", err == ESP_OK ? "PASS" : esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

static int cmd_bootcfg(int argc, char **argv)
{
    esp_err_t err = keypad_boot_config();
    printf("bootcfg: %s\n", err == ESP_OK ? "OK" : esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

static int cmd_config(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: config idle|active\n");
        return 1;
    }
    esp_err_t err;
    if (strcmp(argv[1], "idle") == 0) {
        err = keypad_apply_idle_config();
    } else if (strcmp(argv[1], "active") == 0) {
        err = keypad_apply_active_config();
    } else {
        printf("usage: config idle|active\n");
        return 1;
    }
    printf("config %s: %s\n", argv[1], err == ESP_OK ? "OK" : esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

static int cmd_dump(int argc, char **argv)
{
    printf("GTX312L registers 01h..2Dh:\n");
    for (uint8_t reg = 0x01; reg <= 0x2D; reg++) {
        uint8_t val = 0;
        esp_err_t err = keypad_read_reg(reg, &val);
        if (err != ESP_OK) {
            printf("  %02X: read error (%s)\n", reg, esp_err_to_name(err));
            return 1;
        }
        printf(" %02X:%02X", reg, val);
        if (reg % 6 == 0) printf("\n");
    }
    printf("\nINT line: %d (0=asserted)\n", keypad_get_int_level());
    return 0;
}

static int cmd_read(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: read <reg-hex>\n");
        return 1;
    }
    uint8_t reg = (uint8_t)strtoul(argv[1], NULL, 16);
    uint8_t val = 0;
    esp_err_t err = keypad_read_reg(reg, &val);
    if (err != ESP_OK) {
        printf("%02X: read error (%s)\n", reg, esp_err_to_name(err));
        return 1;
    }
    printf("%02X: %02X\n", reg, val);
    return 0;
}

static int cmd_write(int argc, char **argv)
{
    if (argc != 3) {
        printf("usage: write <reg-hex> <val-hex>\n");
        return 1;
    }
    uint8_t reg = (uint8_t)strtoul(argv[1], NULL, 16);
    uint8_t val = (uint8_t)strtoul(argv[2], NULL, 16);
    esp_err_t err = keypad_write_reg(reg, val);
    printf("%02X <- %02X: %s\n", reg, val, err == ESP_OK ? "OK" : esp_err_to_name(err));
    return err == ESP_OK ? 0 : 1;
}

/** @brief Expand a 12-bit mask into per-key flags for the change printout. */
static void print_mask(uint16_t mask)
{
    if (mask == 0) {
        printf("(none)");
        return;
    }
    bool first = true;
    for (uint8_t bit = 0; bit < 12; bit++) {
        if (mask & (1U << bit)) {
            printf("%s%s(K%d)", first ? "" : " ",
                   keypad_key_label(keypad_key_from_bit(bit)), bit + 1);
            first = false;
        }
    }
}

static int cmd_scan(int argc, char **argv)
{
    return keypad_scan_bus() == ESP_OK ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* Step 2: event engine                                                */

static QueueHandle_t s_event_queue;

static int cmd_events(int argc, char **argv)
{
    int seconds = (argc == 2) ? atoi(argv[1]) : 30;
    if (seconds <= 0) seconds = 30;

    if (s_event_queue == NULL) {
        s_event_queue = xQueueCreate(16, sizeof(keypad_event_t));
        if (s_event_queue == NULL) {
            printf("queue create failed\n");
            return 1;
        }
    }
    xQueueReset(s_event_queue);

    if (keypad_start(s_event_queue) != ESP_OK) {
        printf("engine start failed\n");
        return 1;
    }
    printf("engine running for %d s - touch the pad (events are debounced)\n",
           seconds);

    int64_t deadline = esp_log_timestamp() + (uint32_t)seconds * 1000;
    while (esp_log_timestamp() < deadline) {
        keypad_event_t ev;
        if (xQueueReceive(s_event_queue, &ev, pdMS_TO_TICKS(250)) == pdTRUE) {
            printf("%-8s '%s' t=%lld ms\n", ev.pressed ? "PRESSED" : "RELEASED",
                   keypad_key_label(ev.key), (long long)ev.timestamp_ms);
        }
    }

    esp_err_t err = keypad_stop();
    printf("engine stopped (%s)\n", err == ESP_OK ? "OK" : esp_err_to_name(err));

    keypad_stats_t st;
    if (keypad_get_stats(&st) == ESP_OK) {
        printf("stats: accepted=%lu rejected_short=%lu rejected_ambiguous=%lu\n",
               (unsigned long)st.accepted, (unsigned long)st.rejected_short,
               (unsigned long)st.rejected_ambiguous);
    }
    return err == ESP_OK ? 0 : 1;
}

static int cmd_mask(int argc, char **argv)
{
    if (argc != 2 || (strcmp(argv[1], "on") != 0 && strcmp(argv[1], "off") != 0)) {
        printf("usage: mask on|off\n");
        return 1;
    }
    keypad_mask_touch(strcmp(argv[1], "on") == 0);
    return 0;
}

static int cmd_monitor(int argc, char **argv)
{
    int polls = (argc == 2) ? atoi(argv[1]) : 500;
    if (polls <= 0) polls = 500;

    printf("monitor: %d polls @ %d ms - raw TOUCH_OUT, no debounce\n",
           polls, CONFIG_KEYPAD_POLL_PERIOD_MS);

    uint16_t prev = 0xFFFF;   /* force the first printout */
    int prev_int = -1;
    int64_t t0 = esp_log_timestamp();

    for (int i = 0; i < polls; i++) {
        uint16_t mask = 0;
        esp_err_t err = keypad_read_touch_mask(&mask);
        if (err != ESP_OK) {
            printf("read error: %s\n", esp_err_to_name(err));
            return 1;
        }
        int int_lvl = keypad_get_int_level();

        if (mask != prev || int_lvl != prev_int) {
            printf("t=%6lu ms  TOUCH_OUT=0x%03X  INT=%d  ",
                   (unsigned long)(esp_log_timestamp() - t0), mask, int_lvl);
            print_mask(mask);
            printf("\n");
            prev = mask;
            prev_int = int_lvl;
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_KEYPAD_POLL_PERIOD_MS));
    }
    return 0;
}

void app_main(void)
{
    keypad_config_t cfg = KEYPAD_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(keypad_init(&cfg));

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "keypad>";
    repl_cfg.max_history_len = 8;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    const esp_console_cmd_t cmds[] = {
        { .command = "selftest", .help = "CHIP_ID + SENSITIVITY1 checks",   .func = &cmd_selftest },
        { .command = "scan",     .help = "probe all 7-bit addresses",       .func = &cmd_scan },
        { .command = "bootcfg",  .help = "OEM power-on sequence",           .func = &cmd_bootcfg },
        { .command = "config",   .help = "config idle|active",              .func = &cmd_config },
        { .command = "dump",     .help = "dump registers 01h..2Dh",         .func = &cmd_dump },
        { .command = "read",     .help = "read <reg-hex>",                  .func = &cmd_read },
        { .command = "write",    .help = "write <reg-hex> <val-hex>",       .func = &cmd_write },
        { .command = "monitor",  .help = "monitor [polls] raw TOUCH_OUT",   .func = &cmd_monitor },
        { .command = "events",   .help = "events [seconds] debounced keys", .func = &cmd_events },
        { .command = "mask",     .help = "mask on|off (RF window)",         .func = &cmd_mask },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    ESP_LOGI(TAG, "keypad_test step 2 ready - type 'help' for commands");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
