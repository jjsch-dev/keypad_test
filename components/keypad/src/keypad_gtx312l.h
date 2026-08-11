/**
 * @file keypad_gtx312l.h
 * @brief Low-level driver for the GreenChip GTX312L 12-channel capacitive
 *        touch controller (I2C).
 *
 * All register facts are wire-proven (BP5 sniffer + scope captures, see
 * PROJECT_NOTES.md "Stage 3"). Where the OEM writes undocumented values
 * they are kept as parity blobs and marked as such (decision D2).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit address: fixed 1011 + CHIP_ID=001 -> 0x59 (CTRL pin OPEN on this
 * board, datasheet 8.4). Wire bytes: 0xB2 (W) / 0xB3 (R). */
#define GTX312L_I2C_ADDR          0x59

/* Register map (datasheet) */
#define GTX312L_REG_CHIP_ID       0x01  /* RO; 0xB2 with CTRL=OPEN */
#define GTX312L_REG_TOUCH_OUT1    0x02  /* channels 1-8,  bit n-1 = channel n */
#define GTX312L_REG_TOUCH_OUT2    0x03  /* channels 9-12, bit n-9 = channel n */
#define GTX312L_REG_CH_EN1        0x04  /* reset 0xFF (OEM never writes) */
#define GTX312L_REG_CH_EN2        0x05  /* reset 0x3F (OEM never writes) */
#define GTX312L_REG_MON_RST       0x0A
#define GTX312L_REG_SOFT_RST      0x0B  /* 0 = operation, 1 = sleep */
#define GTX312L_REG_I2C_PU_DIS    0x0C  /* internal I2C pull-ups, on by default */
#define GTX312L_REG_WRITE_LOCK    0x0F  /* magic 0x5A (OEM never writes) */
#define GTX312L_REG_INT_MULTI     0x10  /* bit4 INT_MODE (0=pulse), bit0 MULTI */
#define GTX312L_REG_TOUCH_EXPIRE  0x11  /* reset 0x32 (OEM never writes) */
#define GTX312L_REG_CAL_TIME      0x13  /* OEM idle: 0x05 = 500 ms */
#define GTX312L_REG_SEN_IDLE_TIME 0x14
#define GTX312L_REG_SEN_SUFFIX    0x15
#define GTX312L_REG_BUSY_TO_IDLE  0x17
#define GTX312L_REG_I2B_MODE      0x18
#define GTX312L_REG_SLIDE_MODE    0x19
#define GTX312L_REG_SENSITIVITY1  0x20  /* .. 0x2B = SENSITIVITY12; reset 0x0F */
#define GTX312L_REG_SENSITIVITY12 0x2B

/* Undocumented registers written by the OEM idle burst (parity blobs) */
#define GTX312L_REG_UNK_66        0x66  /* OEM writes 0x02 */
#define GTX312L_REG_UNK_40        0x40  /* OEM writes 8 bytes: 02 01 01 00 02 03 00 02 */

/* CHIP_ID expected value with CTRL pin OPEN (wire-proven) */
#define GTX312L_CHIP_ID_CTRL_OPEN 0xB2

/* Sensitivity semantics (datasheet): LOWER value = MORE sensitive.
 * 0x03 very sensitive .. 0x3F insensitive; recommended range 0x0C..0x3F.
 * OEM deviates below the recommended floor: active 0x08, idle 0x0A. */

typedef enum {
    GTX312L_PRESET_ACTIVE = 0,  /* touch-tracking config (sens, fast scan) */
    GTX312L_PRESET_IDLE,        /* low-power config (sens, 330 ms scan) */
} gtx312l_preset_t;

typedef struct {
    uint8_t sens_active;     /* SENSITIVITY1..12 while tracking a touch */
    uint8_t sens_idle;       /* SENSITIVITY1..12 while idle */
    uint8_t idle_time;       /* SEN_IDLE_TIME for idle scan (OEM: 0x03) */
    uint8_t idle_suffix;     /* SEN_SUFFIX for idle scan   (OEM: 0x03) */
} gtx312l_tune_t;

/**
 * @brief Attach the chip to an existing I2C master bus.
 *
 * The bus itself is owned by the caller (shared with future peripherals).
 */
esp_err_t gtx312l_init(i2c_master_bus_handle_t bus, uint32_t scl_speed_hz);

/**
 * @brief Identity + fresh-POR self-test (mirrors the OEM boot sequence).
 *
 *  - reads CHIP_ID (01h), expects 0xB2  -> proves I2C path + chip identity
 *  - reads SENSITIVITY1 (20h), expects reset value 0x0F -> proves the chip
 *    is in its reset state (a stale value only means the chip was already
 *    configured since POR; it is logged as a warning, not an error).
 */
esp_err_t gtx312l_self_test(void);

esp_err_t gtx312l_read_reg(uint8_t reg, uint8_t *val);
esp_err_t gtx312l_write_reg(uint8_t reg, uint8_t val);

/** @brief Read TOUCH_OUT1/2 (02h/03h) as a 12-bit mask (bit n-1 = channel n). */
esp_err_t gtx312l_read_touch_out(uint16_t *mask12);

/**
 * @brief Write one OEM config preset (wire-proven sequences).
 *
 * ACTIVE = the delta the OEM writes right after the first touch:
 *          SENSITIVITY1..12 = sens_active (single 12-byte frame) and
 *          14h=0x00, 15h=0x01 (fast ~11 ms sensing).
 * IDLE   = the full 10-frame going-to-sleep burst, including the
 *          undocumented 66h / 40h-47h parity blobs, CAL_TIME=500 ms,
 *          SENSITIVITY = sens_idle and 330 ms idle sensing.
 */
esp_err_t gtx312l_apply_preset(gtx312l_preset_t preset, const gtx312l_tune_t *tune);

/**
 * @brief OEM power-on sequence: undocumented magic 0Bh=0x5B + 0Bh=0x00,
 *        then an ACTIVE pass followed by the IDLE pass (as captured at boot).
 */
esp_err_t gtx312l_boot_sequence(const gtx312l_tune_t *tune);

#ifdef __cplusplus
}
#endif
